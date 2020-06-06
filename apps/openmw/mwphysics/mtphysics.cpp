#include <atomic>
#include <mutex>

#include <LinearMath/btThreads.h>
#include <shared_mutex>

#include "components/debug/debuglog.hpp"
#include "components/settings/settings.hpp"

#include "mtphysics.hpp"

namespace MWPhysics
{
    PhysicsTaskScheduler::PhysicsTaskScheduler(PhysicsSystem* physics)
        : btITaskScheduler("physics task scheduler")
        , mNumThreads(Settings::Manager::getInt("solver num threads", "Physics"))
        , mNumJob(0)
        , mNextJob(0)
        , mRunningThreads(0)
        , mShouldStop(false)
        , mPhysics(physics)
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

        mNumJob = iEnd - iBegin;
        mNextJob = iBegin;

        if (mNumJob <= 0)
            return;
        mHasJob.notify_all();

        auto const allJobsDone = [&]()
        {
            // we don't care if we get these values a little bit later, but we do want to reduce contention around them
            return std::atomic_load_explicit(&mRunningThreads, std::memory_order_relaxed) == 0
                && std::atomic_load_explicit(&mNextJob, std::memory_order_relaxed) >= mNumJob;
        };
        mDone.wait(lock, allJobsDone);

        mNumJob = 0;
    }

    void PhysicsTaskScheduler::worker()
    {
        while (!mShouldStop)
        {
            {
                std::unique_lock<std::mutex> lock(mJobsMutex);
                mHasJob.wait(lock, [&]() { return mShouldStop || mNumJob > 0; });
            }

            int job = 0;
            std::atomic_fetch_add(&mRunningThreads, 1);
            while((job = std::atomic_fetch_add(&mNextJob, 1)) < mNumJob)
                mPhysics->applyQueuedMovementActor(job);
            std::atomic_fetch_sub(&mRunningThreads, 1);
            mDone.notify_one();
        }
    }

    SingleThreadPhysicsFallback::SingleThreadPhysicsFallback(PhysicsSystem* physics)
        : mPhysics(physics)
    {}

    void SingleThreadPhysicsFallback::forLoop(int iBegin, int iEnd) const
    {
        for (int i = iBegin; i < iEnd; ++i)
            mPhysics->applyQueuedMovementActor(i);
    }

    void CheckBulletMultithreadingSupport::forLoop(int iBegin, int iEnd) const
    {
#ifdef MULTITHREADED_PHYSICS
        Log(Debug::Warning) << "Bullet was not compiled with multithreading support, physics update will be single threaded";
#endif
    }
}
