#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHMANAGER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHMANAGER_H

#include "asyncnavmeshupdater.hpp"
#include "cachedrecastmeshmanager.hpp"
#include "sharednavmesh.hpp"

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <osg/Vec3f>

#include <map>
#include <memory>

class dtNavMesh;

namespace DetourNavigator
{
    class NavMeshManager
    {
    public:
        NavMeshManager(const Settings& settings);

        bool addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform);

        bool removeObject(std::size_t id);

        void addAgent(const osg::Vec3f& agentHalfExtents);

        void reset(const osg::Vec3f& agentHalfExtents);

        void update(osg::Vec3f playerPosition, const osg::Vec3f& agentHalfExtents);

        SharedNavMesh getNavMesh(const osg::Vec3f& agentHalfExtents) const;

        std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> getNavMeshes() const;

        void wait();

    private:
        std::size_t mRevision = 0;
        const Settings& mSettings;
        CachedRecastMeshManager mRecastMeshManager;
        std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> mCache;
        std::map<osg::Vec3f, std::set<TilePosition>> mChangedTiles;
        AsyncNavMeshUpdater mAsyncNavMeshUpdater;

        std::shared_ptr<NavMeshCacheItem> getCached(const osg::Vec3f& agentHalfExtents) const;

        void addChangedTiles(const btCollisionShape& shape, const btTransform& transform);
    };
}

#endif
