#include "recastmeshmanager.hpp"

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

namespace DetourNavigator
{
    RecastMeshManager::RecastMeshManager(const Settings& settings)
        : mSettings(settings)
        , mMeshBuilder(settings)
    {
    }

    bool RecastMeshManager::addObject(std::size_t id, const btHeightfieldTerrainShape& shape, const btTransform& transform)
    {
        if (!mObjects.insert(std::make_pair(id, Object {&shape, transform})).second)
            return false;
        rebuild();
        mMeshBuilder.addObject(shape, transform);
        return true;
    }

    bool RecastMeshManager::addObject(std::size_t id, const btConcaveShape& shape, const btTransform& transform)
    {
        if (const auto concaveShape = dynamic_cast<const btConcaveShape*>(&shape))
        {
            if (!mObjects.insert(std::make_pair(id, Object {&shape, transform})).second)
                return false;
            rebuild();
            mMeshBuilder.addObject(*concaveShape, transform);
            return true;
        }
        return false;
    }

    bool RecastMeshManager::removeObject(std::size_t id)
    {
        if (!mObjects.erase(id))
            return false;
        mShouldRebuild = true;
        return true;
    }

    std::shared_ptr<RecastMesh> RecastMeshManager::getMesh()
    {
        rebuild();
        return mMeshBuilder.create();
    }

    void RecastMeshManager::rebuild()
    {
        if (mShouldRebuild)
        {
            mMeshBuilder = RecastMeshBuilder(mSettings);
            for (const auto& v : mObjects)
                if (const auto heightField = dynamic_cast<const btHeightfieldTerrainShape*>(v.second.mShape))
                    mMeshBuilder.addObject(*heightField, v.second.mTransform);
                else if (const auto concaveShape = dynamic_cast<const btConcaveShape*>(v.second.mShape))
                    mMeshBuilder.addObject(*concaveShape, v.second.mTransform);
            mShouldRebuild = false;
        }
    }
}
