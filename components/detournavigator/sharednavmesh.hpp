#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_SHAREDNAVMESH_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_SHAREDNAVMESH_H

#include <mutex>
#include <memory>

class dtCrowd;
class dtNavMesh;

namespace DetourNavigator
{
    using NavMeshPtr = std::shared_ptr<dtNavMesh>;
    using CrowdPtr = std::shared_ptr<dtCrowd>;

    class LockedSharedNavMesh
    {
    public:
        LockedSharedNavMesh(std::mutex& mutex, const NavMeshPtr& value, const CrowdPtr& crowd)
            : mLock(new std::lock_guard<std::mutex>(mutex)), mValue(value), mCrowd(crowd)
        {}

        dtNavMesh* operator ->() const
        {
            return mValue.get();
        }

        dtNavMesh& operator *() const
        {
            return *mValue;
        }

        dtNavMesh* get() const
        {
            return mValue.get();
        }

        dtCrowd& crowd() const
        {
            return *mCrowd;
        }

    private:
        std::unique_ptr<const std::lock_guard<std::mutex>> mLock;
        NavMeshPtr mValue;
        CrowdPtr mCrowd;
    };

    class SharedNavMesh
    {
    public:
        SharedNavMesh(const NavMeshPtr& value, const CrowdPtr& crowd)
            : mMutex(std::make_shared<std::mutex>()), mValue(value), mCrowd(crowd)
        {}

        LockedSharedNavMesh lock() const
        {
            return LockedSharedNavMesh(*mMutex, mValue, mCrowd);
        }

    private:
        std::shared_ptr<std::mutex> mMutex;
        NavMeshPtr mValue;
        CrowdPtr mCrowd;
    };
}

#endif
