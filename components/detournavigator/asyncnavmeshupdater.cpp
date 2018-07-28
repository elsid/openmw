#include "asyncnavmeshupdater.hpp"
#include "debug.hpp"
#include "makenavmesh.hpp"
#include "settings.hpp"

#include <DetourCrowd.h>
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <iostream>

namespace
{
    using DetourNavigator::ChangeType;
    using DetourNavigator::TilePosition;

    int getManhattanDistance(const TilePosition& lhs, const TilePosition& rhs)
    {
        return std::abs(lhs.x() - rhs.x()) + std::abs(lhs.y() - rhs.y());
    }

    std::tuple<ChangeType, int, int> makePriority(const TilePosition& position, const ChangeType changeType,
                                                    const TilePosition& playerTile)
    {
        return std::make_tuple(
            changeType,
            getManhattanDistance(position, playerTile),
            getManhattanDistance(position, TilePosition {0, 0})
        );
    }

    void updateAgent(dtCrowd& crowd, const int idx)
    {
        const auto agent = crowd.getEditableAgent(idx);

        if (!agent->active)
            return;

        float nearest[3];
        dtPolyRef ref = 0;
        dtVcopy(nearest, agent->npos);
        const auto status = crowd.getNavMeshQuery()->findNearestPoly(agent->npos, crowd.getQueryHalfExtents(),
            crowd.getFilter(agent->params.queryFilterType), &ref, nearest);
        if (dtStatusFailed(status))
        {
            dtVcopy(nearest, agent->npos);
            ref = 0;
        }

        agent->corridor.reset(ref, nearest);
        agent->boundary.reset();
        agent->partial = false;
        agent->topologyOptTime = 0;
        agent->targetReplanTime = 0;
        agent->nneis = 0;
        dtVset(agent->dvel, 0,0,0);
        dtVset(agent->nvel, 0,0,0);
        dtVset(agent->vel, 0,0,0);
        dtVcopy(agent->npos, nearest);
        agent->desiredSpeed = 0;
        agent->state = ref ? DT_CROWDAGENT_STATE_WALKING : DT_CROWDAGENT_STATE_INVALID;
        agent->targetState = DT_CROWDAGENT_TARGET_NONE;
        agent->active = true;
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
        const std::map<TilePosition, ChangeType>& changedTiles)
    {
        log("post jobs playerTile=", playerTile);
        const std::lock_guard<std::mutex> lock(mMutex);
        mPlayerTile = playerTile;
        for (const auto& changedTile : changedTiles)
        {
            if (mPushed[agentHalfExtents].insert(changedTile.first).second)
                mJobs.push(Job {agentHalfExtents, mNavMeshCacheItem, changedTile.first,
                                makePriority(changedTile.first, changedTile.second, playerTile)});
        }
        log("posted ", mJobs.size(), " jobs");
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

                {
                    const auto locked = job.mNavMeshCacheItem->mValue.lock();
                    auto& crowd = locked.crowd();
                    for (int i = 0, count = crowd.getAgentCount(); i < count; ++i)
                        updateAgent(crowd, i);
                }

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
                    " playerTile=", mPlayerTile,
                    " jobsDone=", ++mJobsDone);
            }
            catch (const std::exception& e)
            {
                log("AsyncNavMeshUpdater::process exception: ", e.what());
            }
        }
        log("stop process jobs");
    }
}
