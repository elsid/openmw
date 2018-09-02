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
    static std::ostream& operator <<(std::ostream& stream, UpdateNavMeshStatus value)
    {
        switch (value)
        {
            case UpdateNavMeshStatus::ignore:
                return stream << "ignore";
            case UpdateNavMeshStatus::removed:
                return stream << "removed";
            case UpdateNavMeshStatus::add:
                return stream << "add";
            case UpdateNavMeshStatus::replaced:
                return stream << "replaced";
        }
        return stream << "unknown";
    }

    AsyncNavMeshUpdater::AsyncNavMeshUpdater(const Settings& settings, TileCachedRecastMeshManager& recastMeshManager)
        : mSettings(settings),
          mRecastMeshManager(recastMeshManager),
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

    void AsyncNavMeshUpdater::post(const osg::Vec3f& agentHalfExtents,
        const std::shared_ptr<NavMeshCacheItem>& mNavMeshCacheItem, const TilePosition& playerTile,
        const std::set<TilePosition>& changedTiles)
    {
        log("post jobs playerTile=", playerTile);
        const std::lock_guard<std::mutex> lock(mMutex);
        mPlayerTile = playerTile;
        for (const auto& changedTile : changedTiles)
        {
            if (mPushed[agentHalfExtents].insert(changedTile).second)
                mJobs.push(Job {agentHalfExtents, mNavMeshCacheItem, changedTile,
                                makePriority(changedTile, playerTile)});
        }
        mHasJob.notify_all();
    }

    void AsyncNavMeshUpdater::wait()
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mDone.wait(lock, [&] { return mJobs.empty(); });
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
                {
                    mDone.notify_all();
                    mHasJob.wait_for(lock, std::chrono::milliseconds(10));
                }
                if (mJobs.empty())
                {
                    mDone.notify_all();
                    continue;
                }
                if (!mStart)
                    mStart = std::chrono::steady_clock::now();
                log("got ", mJobs.size(), " jobs");
                const auto job = mJobs.top();
                mJobs.pop();
                const auto pushed = mPushed.find(job.mAgentHalfExtents);
                pushed->second.erase(job.mChangedTile);
                if (pushed->second.empty())
                    mPushed.erase(pushed);
                const auto playerTile = mPlayerTile;
                lock.unlock();
                log("process job for agent=", job.mAgentHalfExtents);
                using float_milliseconds = std::chrono::duration<float, std::milli>;
                const auto start = std::chrono::steady_clock::now();
                std::string revision;
                std::string recastMeshRevision;
                std::string navMeshRevision;
                if ((mSettings.mEnableWriteNavMeshToFile || mSettings.mEnableWriteRecastMeshToFile)
                        && (mSettings.mEnableRecastMeshFileNameRevision || mSettings.mEnableNavMeshFileNameRevision))
                {
                    revision = "." + std::to_string((std::chrono::steady_clock::now()
                                                     - std::chrono::steady_clock::time_point()).count());
                    if (mSettings.mEnableRecastMeshFileNameRevision)
                        recastMeshRevision = revision;
                    if (mSettings.mEnableNavMeshFileNameRevision)
                        navMeshRevision = revision;
                }
                const auto recastMesh = mRecastMeshManager.getMesh(job.mChangedTile);
                const auto status = updateNavMesh(job.mAgentHalfExtents, recastMesh, job.mChangedTile, playerTile,
                                                  mSettings, *job.mNavMeshCacheItem);
                if (recastMesh && mSettings.mEnableWriteRecastMeshToFile)
                    writeToFile(*recastMesh, mSettings.mRecastMeshPathPrefix + std::to_string(job.mChangedTile.x())
                                + "_" + std::to_string(job.mChangedTile.y()) + "_", recastMeshRevision);
                if (mSettings.mEnableWriteNavMeshToFile)
                    writeToFile(*job.mNavMeshCacheItem->mValue.lock(), mSettings.mNavMeshPathPrefix, navMeshRevision);
                const auto finish = std::chrono::steady_clock::now();
                log("cache updated for agent=", job.mAgentHalfExtents, " status=", status,
                    " generation=", job.mNavMeshCacheItem->mGeneration,
                    " revision=", job.mNavMeshCacheItem->mNavMeshRevision,
                    " time=", std::chrono::duration_cast<float_milliseconds>(finish - start).count(), "ms",
                    " total_time=", std::chrono::duration_cast<float_milliseconds>(finish - *mStart).count(), "ms",
                    " changedTile=", job.mChangedTile,
                    " playerTile=", mPlayerTile);
            }
            catch (const std::exception& e)
            {
                log("AsyncNavMeshUpdater::process exception: ", e.what());
            }
        }
        log("stop process jobs");
    }
}
