#include "recastmeshbuilder.hpp"
#include "settings.hpp"

#include <components/bullethelpers/processtrianglecallback.hpp>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>

namespace DetourNavigator
{
    using BulletHelpers::makeProcessTriangleCallback;

    RecastMeshBuilder::RecastMeshBuilder(const Settings& settings)
        : mSettings(&settings)
    {
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
