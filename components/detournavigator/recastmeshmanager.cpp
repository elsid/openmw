#include "recastmeshmanager.hpp"

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

namespace DetourNavigator
{
    RecastMeshManager::RecastMeshManager(const Settings& settings)
        : mSettings(settings)
        , mMeshBuilder(settings)
    {}

    bool RecastMeshManager::addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform)
    {
        if (!mObjects.insert(std::make_pair(id, Object {&shape, transform})).second)
            return false;
        rebuild();
        return mMeshBuilder.addObject(shape, transform);
    }

    boost::optional<RecastMeshManager::Object> RecastMeshManager::removeObject(std::size_t id)
    {
        const auto object = mObjects.find(id);
        if (object == mObjects.end())
            return boost::none;
        const auto result = object->second;
        mObjects.erase(object);
        mShouldRebuild = true;
        return result;
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
