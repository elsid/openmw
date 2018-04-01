#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_ASYNCNAVMESHUPDATER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_ASYNCNAVMESHUPDATER_H

#include "recastmesh.hpp"
#include "tileposition.hpp"

#include <osg/Vec3f>

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

class dtNavMesh;

namespace DetourNavigator
{
    using NavMeshPtr = std::shared_ptr<dtNavMesh>;

    struct NavMeshCacheItem
    {
        NavMeshPtr mValue;
        std::size_t mRevision;
    };

    class AsyncNavMeshUpdater
    {
    public:
        AsyncNavMeshUpdater(const Settings& settings);
        ~AsyncNavMeshUpdater();

        void post(const osg::Vec3f& agentHalfExtents, const std::shared_ptr<RecastMesh>& recastMesh,
                const std::shared_ptr<NavMeshCacheItem>& mNavMeshCacheItem, std::set<TilePosition>&& changedTiles);

    private:
        struct Job
        {
            osg::Vec3f mAgentHalfExtents;
            std::shared_ptr<RecastMesh> mRecastMesh;
            std::shared_ptr<NavMeshCacheItem> mNavMeshCacheItem;
            std::set<TilePosition> mChangedTiles;
        };

        using Jobs = std::map<osg::Vec3f, std::shared_ptr<Job>>;

        const Settings& mSettings;
        std::atomic_bool mShouldStop;
        std::mutex mMutex;
        std::condition_variable mHasJob;
        Jobs mJobs;
        std::thread mThread;

        void process() throw();
    };
}

#endif
