#include <atomic>
#include <mutex>

#include <LinearMath/btThreads.h>
#include <shared_mutex>

#include "components/debug/debuglog.hpp"
#include "components/settings/settings.hpp"

#include "mtphysics.hpp"

namespace MWPhysics
{
    PhysicsTaskScheduler::PhysicsTaskScheduler()
        : btITaskScheduler("physics task scheduler")
        , mNumThreads(Settings::Manager::getInt("solver num threads", "Physics"))
        , mShouldStop(false)
    {
#ifdef MULTITHREADED_PHYSICS
        mNumThreads = std::max(1, mNumThreads);
#else
        if (mNumThreads != 1)
            Log(Debug::Warning) << "OpenMW compiled without MULTITHREADED_PHYSICS, setting \"solver num threads\" ignored";
        mNumThreads = 1;
#endif
        for (int i = 0; i < mNumThreads; ++i)
            mThreads.emplace_back([&] { worker(); } );
    }

    PhysicsTaskScheduler::~PhysicsTaskScheduler()
    {
        std::unique_lock<std::mutex> lock(mJobsMutex);
        mShouldStop = true;
        mHasJob.notify_all();
        lock.unlock();
        for (auto& thread : mThreads)
            thread.join();
    }

    void PhysicsTaskScheduler::parallelFor(int iBegin, int iEnd, int batchsize, const btIParallelForBody& body)
    {
        std::unique_lock<std::mutex> lock(mJobsMutex);

        for (int i = iBegin; i < iEnd; i += batchsize)
            mJobs.emplace_back(i, std::min(iEnd, i + batchsize), body);

        std::atomic_init(&mJobsDone, 0);
        int jobnum = mJobs.size();
        if (!mJobs.empty())
        {
            mHasJob.notify_all();
            mDone.wait(lock, [&]() { return mJobs.empty() && std::atomic_load(&mJobsDone) == jobnum; });
        }
    }

    void PhysicsTaskScheduler::worker()
    {
        while (!mShouldStop)
        {
            std::unique_lock<std::mutex> lock(mJobsMutex);

            if (!mHasJob.wait_for(lock, std::chrono::milliseconds(10), [&]() { return mShouldStop || !mJobs.empty(); }))
            {
                mDone.notify_one();
            }
            else
            {
                if (!mJobs.empty())
                {
                    auto job = mJobs.back();
                    mJobs.pop_back();
                    lock.unlock();

                    job.mBody->forLoop(job.mBegin, job.mEnd);

                    std::atomic_fetch_add(&mJobsDone, 1);
                }
                mDone.notify_one();
            }

        }
    }

    ApplyQueuedMovementMT::ApplyQueuedMovementMT(PhysicsSystem* physics)
        : mPhysics(physics)
    {}

    void ApplyQueuedMovementMT::forLoop(int iBegin, int iEnd) const
    {
        mPhysics->applyQueuedMovementRange(iBegin, iEnd);
    }

    void CheckBulletMultithreadingSupport::forLoop(int iBegin, int iEnd) const
    {
#ifdef MULTITHREADED_PHYSICS
        Log(Debug::Warning) << "Bullet was not compiled with multithreading support, physics update will be single threaded";
#endif
    }
}
