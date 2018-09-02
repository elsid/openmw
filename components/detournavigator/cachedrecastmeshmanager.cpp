#include "cachedrecastmeshmanager.hpp"
#include "debug.hpp"

namespace DetourNavigator
{
    CachedRecastMeshManager::CachedRecastMeshManager(const Settings& settings, const TileBounds& bounds)
        : mImpl(settings, bounds)
    {}

    bool CachedRecastMeshManager::addObject(std::size_t id, const btCollisionShape& shape,
                                            const btTransform& transform, const AreaType areaType)
    {
        if (!mImpl.addObject(id, shape, transform, areaType))
            return false;
        mCached.reset();
        return true;
    }

    bool CachedRecastMeshManager::updateObject(std::size_t id, const btCollisionShape& shape,
                                               const btTransform& transform, const AreaType areaType)
    {
        if (!mImpl.updateObject(id, shape, transform, areaType))
            return false;
        mCached.reset();
        return true;
    }

    boost::optional<RecastMeshManager::Object> CachedRecastMeshManager::removeObject(std::size_t id)
    {
        const auto object = mImpl.removeObject(id);
        if (object)
            mCached.reset();
        return object;
    }

    std::shared_ptr<RecastMesh> CachedRecastMeshManager::getMesh()
    {
        if (!mCached)
            mCached = mImpl.getMesh();
        return mCached;
    }

    bool CachedRecastMeshManager::isEmpty() const
    {
        return mImpl.isEmpty();
    }
}
