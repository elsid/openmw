#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHMANAGER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVMESHMANAGER_H

#include "asyncnavmeshupdater.hpp"
#include "cachedrecastmeshmanager.hpp"
#include "flags.hpp"
#include "sharednavmesh.hpp"

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <osg/Vec3f>

#include <map>
#include <memory>

class dtNavMesh;
struct dtCrowdAgentParams;

namespace DetourNavigator
{
    class NavMeshManager
    {
    public:
        NavMeshManager(const Settings& settings);

        bool addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                       const AreaType areaType);

        bool updateObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                          const AreaType areaType);

        bool removeObject(std::size_t id);

        void addAgent(const std::size_t id, const osg::Vec3f& position, const osg::Vec3f& agentHalfExtents,
                      const dtCrowdAgentParams& crowdParams);

        void removeAgent(const std::size_t id);

        void updateAgent(const std::size_t id, const osg::Vec3f& position, const float speed);

        void updateAgentId(const std::size_t id, const std::size_t newId);

        void reset(const osg::Vec3f& agentHalfExtents);

        void update(const osg::Vec3f& playerPosition, const osg::Vec3f& agentHalfExtents);

        void updateCrowd(const float duration);

        bool updateAgentTarget(const std::size_t id, osg::Vec3f position, const Flags includeFlags);

        osg::Vec3f getAgentTarget(const std::size_t id) const;

        osg::Vec3f getAgentPosition(const std::size_t id) const;

        SharedNavMesh getNavMesh(const osg::Vec3f& agentHalfExtents) const;

        std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> getNavMeshes() const;

        void wait();

    private:
        struct CrowdAgent
        {
            int mCrowdId;
            osg::Vec3f mAgentHalfExtents;
        };

        const Settings& mSettings;
        TileCachedRecastMeshManager mRecastMeshManager;
        std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> mCache;
        std::map<osg::Vec3f, std::map<TilePosition, ChangeType>> mChangedTiles;
        std::unordered_map<std::size_t, CrowdAgent> mCrowdAgents;
        AsyncNavMeshUpdater mAsyncNavMeshUpdater;
        std::size_t mGenerationCounter = 0;
        boost::optional<TilePosition> mPlayerTile;
        std::size_t mLastRecastMeshManagerRevision = 0;

        std::shared_ptr<NavMeshCacheItem> getCached(const osg::Vec3f& agentHalfExtents) const;

        void addChangedTiles(const btCollisionShape& shape, const btTransform& transform, const ChangeType changeType);
    };
}

#endif
