#include "asyncnavmeshupdater.hpp"
#include "debug.hpp"
#include "makenavmesh.hpp"

#include <iostream>

namespace DetourNavigator
{
    AsyncNavMeshUpdater::AsyncNavMeshUpdater(const Settings& settings)
        : mSettings(settings),
          mMaxRevision(0),
          mShouldStop(),
          mThread([&] { process(); })
    {
    }

    AsyncNavMeshUpdater::~AsyncNavMeshUpdater()
    {
        mShouldStop = true;
        std::unique_lock<std::mutex> lock(mMutex);
        mJobs.clear();
        mHasJob.notify_all();
        lock.unlock();
        mThread.join();
    }

    void AsyncNavMeshUpdater::post(const osg::Vec3f& agentHalfExtents, const std::shared_ptr<RecastMesh>& recastMesh,
                                   const std::shared_ptr<NavMeshCacheItem>& mNavMeshCacheItem)
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        mJobs[agentHalfExtents] = std::make_shared<Job>(Job {agentHalfExtents, recastMesh, mNavMeshCacheItem});
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
                const auto job = mJobs.begin()->second;
                mJobs.erase(mJobs.begin());
                lock.unlock();
                mMaxRevision = std::max(job->mNavMeshCacheItem->mRevision, mMaxRevision);
                log("process job for agent=", job->mAgentHalfExtents,
                    " revision=", job->mNavMeshCacheItem->mRevision,
                    " max_revision=", mMaxRevision);
                if (job->mNavMeshCacheItem->mRevision < mMaxRevision)
                    continue;
                using float_milliseconds = std::chrono::duration<float, std::milli>;
                const auto start = std::chrono::steady_clock::now();
#ifdef OPENMW_WRITE_TO_FILE
                const auto revision = std::to_string(std::time(nullptr));
                writeToFile(*job->mRecastMesh, revision);
#endif
                job->mNavMeshCacheItem->mValue = makeNavMesh(job->mAgentHalfExtents, *job->mRecastMesh, mSettings);
#ifdef OPENMW_WRITE_TO_FILE
                writeToFile(*job->mNavMeshCacheItem->mValue, revision);
#endif
                const auto finish = std::chrono::steady_clock::now();
                log("cache updated for agent=", job->mAgentHalfExtents,
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
