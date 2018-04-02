#include "asyncnavmeshupdater.hpp"
#include "debug.hpp"
#include "makenavmesh.hpp"
#include "settings.hpp"

#include <iostream>

namespace
{
    using DetourNavigator::TilePosition;

    int getDistance(const TilePosition& lhs, const TilePosition& rhs)
    {
        return std::abs(lhs.x() - rhs.x()) + std::abs(lhs.y() - rhs.y());
    }

    std::pair<int, int> makePriority(const TilePosition& changedTile, const TilePosition& playerTile)
    {
        return std::make_pair(getDistance(changedTile, playerTile), getDistance(changedTile, TilePosition {0, 0}));
    }
}

namespace DetourNavigator
{
    AsyncNavMeshUpdater::AsyncNavMeshUpdater(const Settings& settings)
        : mSettings(settings),
          mShouldStop(),
          mThread([&] { process(); })
    {
    }

    AsyncNavMeshUpdater::~AsyncNavMeshUpdater()
    {
        mShouldStop = true;
        std::unique_lock<std::mutex> lock(mMutex);
        mJobs = decltype(mJobs)();
        mHasJob.notify_all();
        lock.unlock();
        mThread.join();
    }

    void AsyncNavMeshUpdater::post(const osg::Vec3f& agentHalfExtents, const std::shared_ptr<RecastMesh>& recastMesh,
        const std::shared_ptr<NavMeshCacheItem>& mNavMeshCacheItem, const TilePosition& playerTile,
        const std::set<TilePosition>& changedTiles)
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        for (const auto& changedTile : changedTiles)
        {
            mJobs.push(Job {agentHalfExtents, recastMesh, mNavMeshCacheItem, changedTile,
                            makePriority(changedTile, playerTile)});
        }
        mHasJob.notify_all();
    }

    void AsyncNavMeshUpdater::process() throw()
    {
        log("start process jobs");
        while (!mShouldStop)
        {
            try
            {
                std::unique_lock<std::mutex> lock(mMutex);
                if (mJobs.empty())
                    mHasJob.wait_for(lock, std::chrono::milliseconds(10));
                if (mJobs.empty())
                    continue;
                log("got ", mJobs.size(), " jobs");
                const auto job = mJobs.top();
                mJobs.pop();
                lock.unlock();
                log("process job for agent=", job.mAgentHalfExtents);
                using float_milliseconds = std::chrono::duration<float, std::milli>;
                const auto start = std::chrono::steady_clock::now();
                std::string revision;
                if (mSettings.mEnableWriteNavMeshToFile || mSettings.mEnableWriteRecastMeshToFile)
                    revision = std::to_string((std::chrono::steady_clock::now() - std::chrono::steady_clock::time_point()).count());
                if (mSettings.mEnableWriteRecastMeshToFile)
                    writeToFile(*job.mRecastMesh, mSettings.mRecastMeshPathPrefix, revision);
                updateNavMesh(job.mAgentHalfExtents, *job.mRecastMesh, job.mChangedTile, mSettings,
                              job.mNavMeshCacheItem->mValue);
                if (mSettings.mEnableWriteNavMeshToFile)
                    writeToFile(*job.mNavMeshCacheItem->mValue.lock(), mSettings.mNavMeshPathPrefix, revision);
                const auto finish = std::chrono::steady_clock::now();
                log("cache updated for agent=", job.mAgentHalfExtents,
                    " time=", std::chrono::duration_cast<float_milliseconds>(finish - start).count(), "ms");
            }
            catch (const std::exception& e)
            {
                log("AsyncNavMeshUpdater::process exception: ", e.what());
            }
        }
        log("stop process jobs");
    }
}
