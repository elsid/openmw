#include "recastmeshbuilder.hpp"
#include "chunkytrimesh.hpp"
#include "debug.hpp"
#include "settings.hpp"

#include <components/bullethelpers/processtrianglecallback.hpp>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <LinearMath/btTransform.h>

#include <algorithm>

namespace DetourNavigator
{
    using BulletHelpers::makeProcessTriangleCallback;

    RecastMeshBuilder::RecastMeshBuilder(const Settings& settings, const TileBounds& bounds)
        : mSettings(settings)
        , mBounds(bounds)
    {
        mBounds.mMin /= mSettings.mRecastScaleFactor;
        mBounds.mMax /= mSettings.mRecastScaleFactor;
    }

    bool RecastMeshBuilder::addObject(const btCollisionShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        if (shape.isCompound())
        {
            return addObject(static_cast<const btCompoundShape&>(shape), transform, areaType);
        }
        else if (shape.getShapeType() == TERRAIN_SHAPE_PROXYTYPE)
        {
            addObject(static_cast<const btHeightfieldTerrainShape&>(shape), transform, areaType);
            return true;
        }
        else if (shape.isConcave())
        {
            addObject(static_cast<const btConcaveShape&>(shape), transform, areaType);
            return true;
        }
        else if (shape.getShapeType() == BOX_SHAPE_PROXYTYPE)
        {
            addObject(static_cast<const btBoxShape&>(shape), transform, areaType);
            return true;
        }
        // TODO: support more Bullet shapes if required
        log("ignore add to RecastMesh object with shape=", BroadphaseNativeTypes(shape.getShapeType()));
        return false;
    }

    bool RecastMeshBuilder::addObject(const btCompoundShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        bool result = false;
        for (int i = 0, num = shape.getNumChildShapes(); i < num; ++i)
            result |= addObject(*shape.getChildShape(i), transform * shape.getChildTransform(i), areaType);
        return result;
    }

    void RecastMeshBuilder::addObject(const btConcaveShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        return addObject(shape, transform, makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
        {
            if (isUnderwater(triangle, transform))
                return;
            for (std::size_t i = 3; i > 0; --i)
                addTriangleVertex(transform(triangle[i - 1]) * mSettings.mRecastScaleFactor);
            mAreaTypes.push_back(areaType);
        }));
    }

    void RecastMeshBuilder::addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        return addObject(shape, transform, makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
        {
            if (isUnderwater(triangle, transform))
                return;
            for (std::size_t i = 0; i < 3; ++i)
                addTriangleVertex(transform(triangle[i]) * mSettings.mRecastScaleFactor);
            mAreaTypes.push_back(areaType);
        }));
    }

    void RecastMeshBuilder::addObject(const btBoxShape& shape, const btTransform& transform, const AreaType areaType)
    {
        if (isUnderwater(shape, transform))
            return;

        const auto indexOffset = static_cast<int>(mVertices.size() / 3);

        for (int vertex = 0, count = shape.getNumVertices(); vertex < count; ++vertex)
        {
            btVector3 position;
            shape.getVertex(vertex, position);
            addVertex(transform(position) * mSettings.mRecastScaleFactor);
        }

        const std::array<int, 36> indices {{
            0, 2, 3,
            3, 1, 0,
            0, 4, 6,
            6, 2, 0,
            0, 1, 5,
            5, 4, 0,
            7, 5, 1,
            1, 3, 7,
            7, 3, 2,
            2, 6, 7,
            7, 6, 4,
            4, 5, 7,
        }};

        std::transform(indices.begin(), indices.end(), std::back_inserter(mIndices),
            [&] (int index) { return index + indexOffset; });

        std::generate_n(std::back_inserter(mAreaTypes), 12, [=] { return areaType; });
    }

    void RecastMeshBuilder::addWater(const int cellSize, const btScalar level, const btTransform& transform)
    {
        const auto halfCellSize = cellSize / 2;
        Water water;
        water.mHalfCellSize = halfCellSize;
        water.mLevel = level;
        water.mInversedTransform = transform.inverse();
        mWater.push_back(water);

        const auto indexOffset = static_cast<int>(mVertices.size() / 3);

        const std::array<btVector3, 4> vertices {{
            btVector3(-halfCellSize, -halfCellSize, 0),
            btVector3(-halfCellSize, halfCellSize, 0),
            btVector3(halfCellSize, halfCellSize, 0),
            btVector3(halfCellSize, -halfCellSize, 0),
        }};

        std::for_each(vertices.begin(), vertices.end(),
            [&] (const btVector3& vertex)
            {
                const auto transformed = transform(vertex);
                addVertex(btVector3(transformed.x(), transformed.y(), level) * mSettings.mRecastScaleFactor);
            });

        const std::array<int, 6> indices {{
            0, 1, 2,
            0, 2, 3,
        }};

        std::transform(indices.begin(), indices.end(), std::back_inserter(mIndices),
            [&] (const int index) { return index + indexOffset; });

        std::generate_n(std::back_inserter(mAreaTypes), 2, [] { return AreaType_water; });
    }

    std::shared_ptr<RecastMesh> RecastMeshBuilder::create() const
    {
        return std::make_shared<RecastMesh>(mIndices, mVertices, mAreaTypes, mSettings);
    }

    void RecastMeshBuilder::reset()
    {
        mIndices.clear();
        mVertices.clear();
        mAreaTypes.clear();
        mWater.clear();
    }

    void RecastMeshBuilder::addObject(const btConcaveShape& shape, const btTransform& transform,
                                      btTriangleCallback&& callback)
    {
        btVector3 aabbMin;
        btVector3 aabbMax;
        shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
        const btVector3 boundsMinMin(mBounds.mMin.x(), mBounds.mMin.y(), 0);
        const btVector3 boundsMinMax(mBounds.mMin.x(), mBounds.mMax.y(), 0);
        const btVector3 boundsMaxMin(mBounds.mMax.x(), mBounds.mMin.y(), 0);
        const btVector3 boundsMaxMax(mBounds.mMax.x(), mBounds.mMax.y(), 0);
        const auto inversedTransform = transform.inverse();
        const auto localBoundsMinMin = inversedTransform(boundsMinMin);
        const auto localBoundsMinMax = inversedTransform(boundsMinMax);
        const auto localBoundsMaxMin = inversedTransform(boundsMaxMin);
        const auto localBoundsMaxMax = inversedTransform(boundsMaxMax);
        aabbMin.setX(std::min({localBoundsMinMin.x(), localBoundsMinMax.x(),
                               localBoundsMaxMin.x(), localBoundsMaxMax.x()}));
        aabbMin.setY(std::min({localBoundsMinMin.y(), localBoundsMinMax.y(),
                               localBoundsMaxMin.y(), localBoundsMaxMax.y()}));
        aabbMax.setX(std::max({localBoundsMinMin.x(), localBoundsMinMax.x(),
                               localBoundsMaxMin.x(), localBoundsMaxMax.x()}));
        aabbMax.setY(std::max({localBoundsMinMin.y(), localBoundsMinMax.y(),
                               localBoundsMaxMin.y(), localBoundsMaxMax.y()}));
        shape.processAllTriangles(&callback, aabbMin, aabbMax);
    }

    void RecastMeshBuilder::addTriangleVertex(const btVector3& worldPosition)
    {
        mIndices.push_back(int(mVertices.size() / 3));
        addVertex(worldPosition);
    }

    void RecastMeshBuilder::addVertex(const btVector3& worldPosition)
    {
        mVertices.push_back(worldPosition.x());
        mVertices.push_back(worldPosition.z());
        mVertices.push_back(worldPosition.y());
    }

    bool RecastMeshBuilder::isUnderwater(const btVector3* triangle, const btTransform& transform) const
    {
        return std::all_of(triangle, triangle + 3,
            [&] (const btVector3& vertex) { return isUnderwaterWater(transform(vertex)); });
    }

    bool RecastMeshBuilder::isUnderwater(const btBoxShape& box, const btTransform& transform) const
    {
        bool result = true;
        for (int vertex = 0, count = box.getNumVertices(); result && vertex < count; ++vertex)
        {
            btVector3 position;
            box.getVertex(vertex, position);
            result = isUnderwaterWater(transform(position));
        }
        return result;
    }

    bool RecastMeshBuilder::isUnderwaterWater(const btVector3& position) const
    {
        return std::any_of(mWater.begin(), mWater.end(), [&] (const Water& water)
        {
            const auto localPosition = water.mInversedTransform(position);
            return -water.mHalfCellSize <= localPosition.x() && localPosition.x() <= water.mHalfCellSize
                && -water.mHalfCellSize <= localPosition.y() && localPosition.y() <= water.mHalfCellSize
                && position.z() <= water.mLevel;
        });
    }
}
