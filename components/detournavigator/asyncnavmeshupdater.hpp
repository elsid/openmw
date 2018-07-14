#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_ASYNCNAVMESHUPDATER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_ASYNCNAVMESHUPDATER_H

#include "navmeshcacheitem.hpp"
#include "tilecachedrecastmeshmanager.hpp"
#include "tileposition.hpp"

#include <osg/Vec3f>

#include <boost/optional.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <thread>

class dtNavMesh;

namespace DetourNavigator
{
    enum class ChangeType
    {
        remove = 0,
        mixed = 1,
        add = 2,
    };

    class AsyncNavMeshUpdater
    {
    public:
        AsyncNavMeshUpdater(const Settings& settings, TileCachedRecastMeshManager& recastMeshManager);
        ~AsyncNavMeshUpdater();

        void post(const osg::Vec3f& agentHalfExtents, const std::shared_ptr<NavMeshCacheItem>& mNavMeshCacheItem,
                  const TilePosition& playerTile, const std::map<TilePosition, ChangeType>& changedTiles);

        void wait();

    private:
        struct Job
        {
            osg::Vec3f mAgentHalfExtents;
            std::shared_ptr<NavMeshCacheItem> mNavMeshCacheItem;
            TilePosition mChangedTile;
            std::tuple<ChangeType, int, int> mPriority;

            friend inline bool operator <(const Job& lhs, const Job& rhs)
            {
                return lhs.mPriority > rhs.mPriority;
            }
        };

        using Jobs = std::priority_queue<Job, std::deque<Job>>;

        const Settings& mSettings;
        TileCachedRecastMeshManager& mRecastMeshManager;
        std::atomic_bool mShouldStop;
        std::mutex mMutex;
        std::condition_variable mHasJob;
        std::condition_variable mDone;
        Jobs mJobs;
        std::map<osg::Vec3f, std::set<TilePosition>> mPushed;
        boost::optional<std::chrono::steady_clock::time_point> mStart;
        TilePosition mPlayerTile;
        std::atomic_size_t mJobsDone;
        std::thread mThread;

        void process() throw();
    };
}

#endif
