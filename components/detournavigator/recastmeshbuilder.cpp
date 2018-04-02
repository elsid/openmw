#include "recastmeshbuilder.hpp"
#include "chunkytrimesh.hpp"
#include "debug.hpp"
#include "settings.hpp"

#include <components/bullethelpers/processtrianglecallback.hpp>

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

namespace DetourNavigator
{
    using BulletHelpers::makeProcessTriangleCallback;

    RecastMeshBuilder::RecastMeshBuilder(const Settings& settings)
        : mSettings(&settings)
    {}

    bool RecastMeshBuilder::addObject(const btCollisionShape& shape, const btTransform& transform)
    {
        if (shape.isCompound())
        {
            return addObject(static_cast<const btCompoundShape&>(shape), transform);
        }
        else if (shape.getShapeType() == TERRAIN_SHAPE_PROXYTYPE)
        {
            addObject(static_cast<const btHeightfieldTerrainShape&>(shape), transform);
            return true;
        }
        else if (shape.isConcave())
        {
            addObject(static_cast<const btConcaveShape&>(shape), transform);
            return true;
        }
        // TODO: support more Bullet shapes if required
        log("ignore add to RecastMesh object with shape=", BroadphaseNativeTypes(shape.getShapeType()));
        return false;
    }

    bool RecastMeshBuilder::addObject(const btCompoundShape& shape, const btTransform& transform)
    {
        bool result = false;
        for (int i = 0, num = shape.getNumChildShapes(); i < num; ++i)
            result |= addObject(*shape.getChildShape(i), transform * shape.getChildTransform(i));
        return result;
    }

    void RecastMeshBuilder::addObject(const btConcaveShape& shape, const btTransform& transform)
    {
        return addObject(shape, makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
        {
            for (std::size_t i = 3; i > 0; --i)
                addVertex(transform(triangle[i - 1]) * mSettings->mRecastScaleFactor);
        }));
    }

    void RecastMeshBuilder::addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform)
    {
        return addObject(shape, makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
        {
            for (std::size_t i = 0; i < 3; ++i)
                addVertex(transform(triangle[i]) * mSettings->mRecastScaleFactor);
        }));
    }

    std::shared_ptr<RecastMesh> RecastMeshBuilder::create() const
    {
        return std::make_shared<RecastMesh>(mIndices, mVertices, *mSettings);
    }

    void RecastMeshBuilder::addObject(const btConcaveShape& shape, btTriangleCallback&& callback)
    {
        btVector3 aabbMin;
        btVector3 aabbMax;
        shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
        shape.processAllTriangles(&callback, aabbMin, aabbMax);
    }

    void RecastMeshBuilder::addVertex(const btVector3& worldPosition)
    {
        mIndices.push_back(int(mIndices.size()));
        mVertices.push_back(worldPosition.x());
        mVertices.push_back(worldPosition.z());
        mVertices.push_back(worldPosition.y());
    }
}
