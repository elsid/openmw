#ifndef OPENMW_MWPHYSICS_MTPHYSICS_H
#define OPENMW_MWPHYSICS_MTPHYSICS_H

#include <condition_variable>
#include <thread>
#include <shared_mutex>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <LinearMath/btThreads.h>

#include "physicssystem.hpp"

class btIParallelSumBody; // need to compile with bullet < 2.88

namespace MWPhysics
{
    /* The purpose of these 2 classes is to make OpenMW works with Bullet compiled with either single or multithread support.
       At runtime, Bullet resolve the call to btParallelFor() to:
       - btITaskScheduler::parallelFor() if bullet is multithreaded
       - btIParallelForBody::forLoop() if bullet is singlethreaded.

       NOTE: From Bullet 2.88, there is a btDefaultTaskScheduler(), that returns NULL if multithreading is not supported.
       It might be worth considering to simplify the API once OpenMW stops supporting 2.87.
    */
    class PhysicsTaskScheduler : public btITaskScheduler
    {
        public:
            PhysicsTaskScheduler();
            ~PhysicsTaskScheduler() override;
            int getMaxNumThreads() const override { return mNumThreads; };
            int getNumThreads() const override { return mNumThreads; };
            void setNumThreads(int numThreads) override {};

            void parallelFor(int iBegin, int iEnd, int batchsize, const btIParallelForBody& body) override;

            // stub implementation needed to compile with bullet >= 2.88. Method doesn't exists in < 2.87 so we can't use override keyword
            btScalar parallelSum(int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body) { return {}; };

        private:
            struct Job
            {
                int mBegin;
                int mEnd;
                const btIParallelForBody* mBody;

                Job(int iBegin, int iEnd, const btIParallelForBody& body)
                    : mBegin(iBegin), mEnd(iEnd), mBody(&body)
                {};
            };
            void worker();

            int mNumThreads;
            std::atomic_int mJobsDone;
            bool mShouldStop;
            std::vector<Job> mJobs;
            std::vector<std::thread> mThreads;

            mutable std::mutex mJobsMutex;
            std::condition_variable mHasJob;
            std::condition_variable_any mDone;
    };

    class ApplyQueuedMovementMT : public btIParallelForBody
    {
        public:
            explicit ApplyQueuedMovementMT(PhysicsSystem* physics);
            void forLoop(int iBegin, int iEnd) const override;
        private:
            PhysicsSystem* mPhysics;
    };

    class CheckBulletMultithreadingSupport : public btIParallelForBody
    {
        public:
            CheckBulletMultithreadingSupport() = default;
            // dummy method whose purpose is to check support for threading in Bullet.
            void forLoop(int iBegin, int iEnd) const override;
    };
}
#endif
