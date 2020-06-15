#include "recastmeshbuilder.hpp"
#include "chunkytrimesh.hpp"
#include "debug.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"
#include "exceptions.hpp"

#include <components/bullethelpers/transformboundingbox.hpp>
#include <components/bullethelpers/processtrianglecallback.hpp>
#include <components/misc/convert.hpp>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btAabbUtil2.h>

#include <algorithm>
#include <tuple>

namespace DetourNavigator
{
    using BulletHelpers::makeProcessTriangleCallback;

    namespace
    {
        template <std::size_t N>
        inline bool operator <(const std::array<btVector3, N>& lhs, const std::array<btVector3, N>& rhs)
        {
            return std::lexicographical_compare(lhs.begin(), lhs.begin(), rhs.begin(), rhs.begin(),
                [] (const btVector3& lhs, const btVector3& rhs) {
                    return std::tie(lhs.x(), lhs.y(), lhs.z()) < std::tie(rhs.x(), rhs.y(), rhs.z());
                });
        }
    }

    RecastMeshBuilder::RecastMeshBuilder(const Settings& settings, const TileBounds& bounds)
        : mSettings(settings)
        , mBounds(bounds)
    {
        mBounds.mMin /= mSettings.get().mRecastScaleFactor;
        mBounds.mMax /= mSettings.get().mRecastScaleFactor;
    }

    void RecastMeshBuilder::addObject(const btCollisionShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        if (shape.isCompound())
            return addObject(static_cast<const btCompoundShape&>(shape), transform, areaType);
        else if (shape.getShapeType() == TERRAIN_SHAPE_PROXYTYPE)
            return addObject(static_cast<const btHeightfieldTerrainShape&>(shape), transform, areaType);
        else if (shape.isConcave())
            return addObject(static_cast<const btConcaveShape&>(shape), transform, areaType);
        else if (shape.getShapeType() == BOX_SHAPE_PROXYTYPE)
            return addObject(static_cast<const btBoxShape&>(shape), transform, areaType);
        std::ostringstream message;
        message << "Unsupported shape type: " << BroadphaseNativeTypes(shape.getShapeType());
        throw InvalidArgument(message.str());
    }

    void RecastMeshBuilder::addObject(const btCompoundShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        for (int i = 0, num = shape.getNumChildShapes(); i < num; ++i)
            addObject(*shape.getChildShape(i), transform * shape.getChildTransform(i), areaType);
    }

    void RecastMeshBuilder::addObject(const btConcaveShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        return addObject(shape, transform, makeProcessTriangleCallback([&] (btVector3* vertices, int, int)
        {
            Triangle triangle;
            triangle.mIndex = mTriangles.size();
            triangle.mAreaType = areaType;
            std::transform(vertices, vertices + sTriangleVerticesNum, triangle.mVertices.rbegin(),
                           ToNavMeshCoordinates<btVector3> {mSettings});
            mTriangles.push_back(triangle);
        }));
    }

    void RecastMeshBuilder::addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        return addObject(shape, transform, makeProcessTriangleCallback([&] (btVector3* vertices, int, int)
        {
            Triangle triangle;
            triangle.mIndex = mTriangles.size();
            triangle.mAreaType = areaType;
            std::transform(vertices, vertices + sTriangleVerticesNum, triangle.mVertices.begin(),
                           ToNavMeshCoordinates<btVector3> {mSettings});
            mTriangles.push_back(triangle);
        }));
    }

    void RecastMeshBuilder::addObject(const btBoxShape& shape, const btTransform& transform, const AreaType areaType)
    {
        Box box;
        box.mIndex = mBoxes.size();
        box.mAreaType = areaType;
        for (std::size_t i = 0; i < box.mVertices.size(); ++i)
        {
            btVector3 position;
            shape.getVertex(static_cast<int>(i), position);
            box.mVertices[i] = toNavMeshCoordinates(mSettings, transform(position));
        }
        mBoxes.push_back(box);
    }

    void RecastMeshBuilder::addWater(const int cellSize, const btTransform& transform)
    {
        mWater.push_back(RecastMesh::Water {cellSize, transform});
    }

    std::shared_ptr<RecastMesh> RecastMeshBuilder::create(std::size_t generation, std::size_t revision)
    {
        std::sort(mTriangles.begin(), mTriangles.end(), [] (const Triangle& lhs, const Triangle& rhs)
        {
            return std::tie(lhs.mVertices, lhs.mAreaType, lhs.mIndex)
                < std::tie(rhs.mVertices, rhs.mAreaType, rhs.mIndex);
        });

        std::sort(mBoxes.begin(), mBoxes.end(), [] (const Box& lhs, const Box& rhs)
        {
            return std::tie(lhs.mVertices, lhs.mAreaType, lhs.mIndex)
                < std::tie(rhs.mVertices, rhs.mAreaType, rhs.mIndex);
        });

        std::vector<int> indices;
        indices.reserve(mTriangles.size() * sTriangleVerticesNum + mBoxes.size() * sBoxIndicesNum);

        std::vector<float> vertices;
        vertices.reserve((mTriangles.size() * sTriangleVerticesNum + mBoxes.size() * sBoxVerticesNum) * sCoordinatesNum);

        std::vector<AreaType> areaTypes;
        areaTypes.reserve(mTriangles.size() + mBoxes.size() * sBoxTrianglesNum);

        const auto addVertex = [&] (const btVector3& position)
        {
            vertices.push_back(position.x());
            vertices.push_back(position.y());
            vertices.push_back(position.z());
        };

        for (const auto& triangle : mTriangles)
        {
            areaTypes.push_back(triangle.mAreaType);
            for (std::size_t i = 0; i < triangle.mVertices.size(); ++i)
            {
                indices.push_back(static_cast<int>(vertices.size() / sCoordinatesNum));
                addVertex(triangle.mVertices[i]);
            }
        }

        for (const auto& box : mBoxes)
        {
            const auto indexOffset = static_cast<int>(vertices.size() / sCoordinatesNum);

            for (std::size_t i = 0; i < box.mVertices.size(); ++i)
                addVertex(box.mVertices[i]);

            const std::array<int, sBoxIndicesNum> boxIndices {{
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

            std::transform(boxIndices.begin(), boxIndices.end(), std::back_inserter(indices),
                [&] (int index) { return index + indexOffset; });

            std::generate_n(std::back_inserter(areaTypes), sBoxTrianglesNum, [&] { return box.mAreaType; });
        }

        return std::make_shared<RecastMesh>(generation, revision, std::move(indices), std::move(vertices),
            std::move(areaTypes), mWater, mSettings.get().mTrianglesPerChunk);
    }

    void RecastMeshBuilder::reset()
    {
        mTriangles.clear();
        mBoxes.clear();
        mWater.clear();
    }

    void RecastMeshBuilder::addObject(const btConcaveShape& shape, const btTransform& transform,
                                      btTriangleCallback&& callback)
    {
        btVector3 aabbMin;
        btVector3 aabbMax;

        shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

        const btVector3 boundsMin(mBounds.mMin.x(), mBounds.mMin.y(),
            -std::numeric_limits<btScalar>::max() * std::numeric_limits<btScalar>::epsilon());
        const btVector3 boundsMax(mBounds.mMax.x(), mBounds.mMax.y(),
            std::numeric_limits<btScalar>::max() * std::numeric_limits<btScalar>::epsilon());

        auto wrapper = makeProcessTriangleCallback([&] (btVector3* triangle, int partId, int triangleIndex)
        {
            std::array<btVector3, sTriangleVerticesNum> transformed;
            for (std::size_t i = 0; i < transformed.size(); ++i)
                transformed[i] = transform(triangle[i]);
            if (TestTriangleAgainstAabb2(transformed.data(), boundsMin, boundsMax))
                callback.processTriangle(transformed.data(), partId, triangleIndex);
        });

        shape.processAllTriangles(&wrapper, aabbMin, aabbMax);
    }

    void RecastMeshBuilder::addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform,
                                      btTriangleCallback&& callback)
    {
        using BulletHelpers::transformBoundingBox;

        btVector3 aabbMin;
        btVector3 aabbMax;

        shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

        transformBoundingBox(transform, aabbMin, aabbMax);

        aabbMin.setX(std::max(static_cast<btScalar>(mBounds.mMin.x()), aabbMin.x()));
        aabbMin.setX(std::min(static_cast<btScalar>(mBounds.mMax.x()), aabbMin.x()));
        aabbMin.setY(std::max(static_cast<btScalar>(mBounds.mMin.y()), aabbMin.y()));
        aabbMin.setY(std::min(static_cast<btScalar>(mBounds.mMax.y()), aabbMin.y()));

        aabbMax.setX(std::max(static_cast<btScalar>(mBounds.mMin.x()), aabbMax.x()));
        aabbMax.setX(std::min(static_cast<btScalar>(mBounds.mMax.x()), aabbMax.x()));
        aabbMax.setY(std::max(static_cast<btScalar>(mBounds.mMin.y()), aabbMax.y()));
        aabbMax.setY(std::min(static_cast<btScalar>(mBounds.mMax.y()), aabbMax.y()));

        transformBoundingBox(transform.inverse(), aabbMin, aabbMax);

        auto wrapper = makeProcessTriangleCallback([&] (btVector3* triangle, int partId, int triangleIndex)
        {
            std::array<btVector3, sTriangleVerticesNum> transformed;
            for (std::size_t i = 0; i < transformed.size(); ++i)
                transformed[i] = transform(triangle[i]);
            callback.processTriangle(transformed.data(), partId, triangleIndex);
        });

        shape.processAllTriangles(&wrapper, aabbMin, aabbMax);
    }
}
