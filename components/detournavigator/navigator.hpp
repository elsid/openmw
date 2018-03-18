#pragma once

#include <DetourNavMesh.h>

#include <BulletCollision/CollisionShapes/btTriangleCallback.h>

#include <LinearMath/btTransform.h>

#include <osg/Vec3f>

#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

class btCollisionShape;
class btConcaveShape;
class btHeightfieldTerrainShape;
class dtNavMesh;

namespace DetourNavigator
{
    struct NavigatorException : std::runtime_error
    {
        NavigatorException(const std::string& message) : std::runtime_error(message) {}
        NavigatorException(const char* message) : std::runtime_error(message) {}
    };

    class RecastMesh
    {
    public:
        RecastMesh(std::vector<int> indices, std::vector<float> vertices)
            : mIndices(std::move(indices))
            , mVertices(std::move(vertices))
        {}

        const std::vector<int>& getIndices() const
        {
            return mIndices;
        }

        const std::vector<float>& getVertices() const
        {
            return mVertices;
        }

        std::size_t getVerticesCount() const
        {
            return mIndices.size();
        }

        std::size_t getTrianglesCount() const
        {
            return mIndices.size() / 3;
        }

    private:
        std::vector<int> mIndices;
        std::vector<float> mVertices;
    };

    template <class Impl>
    class ProcessTriangleCallback : public btTriangleCallback
    {
    public:
        ProcessTriangleCallback(Impl impl)
            : mImpl(std::move(impl))
        {}

        void processTriangle(btVector3* triangle, int partId, int triangleIndex) override final
        {
            return mImpl(triangle, partId, triangleIndex);
        }

    private:
        Impl mImpl;
    };

    template <class Impl>
    ProcessTriangleCallback<typename std::decay<Impl>::type> makeProcessTriangleCallback(Impl&& impl)
    {
        return ProcessTriangleCallback<typename std::decay<Impl>::type>(std::forward<Impl>(impl));
    }

    class RecastMeshBuilder
    {
    public:
        RecastMeshBuilder& addObject(const btConcaveShape& shape, const btTransform& transform);

        RecastMeshBuilder& addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform);

        RecastMesh create() const;

    private:
        std::vector<int> mIndices;
        std::vector<float> mVertices;

        RecastMeshBuilder& addObject(const btConcaveShape& shape, btTriangleCallback& callback);

        void addVertex(const btVector3& worldPosition);
    };

    class RecastMeshManager
    {
    public:
        bool addObject(std::size_t id, const btHeightfieldTerrainShape& shape, const btTransform& transform);

        bool addObject(std::size_t id, const btConcaveShape& shape, const btTransform& transform);

        bool removeObject(std::size_t id);

        RecastMesh getMesh();

    private:
        struct Object
        {
            const btCollisionShape* mShape;
            btTransform mTransform;
        };

        RecastMeshBuilder mMeshBuilder;
        std::unordered_map<std::size_t, Object> mObjects;

        void rebuild();
    };

    class RecastMeshManagerCache
    {
    public:
        template <class T>
        bool addObject(std::size_t id, const T& shape, const btTransform& transform)
        {
            if (!mImpl.addObject(id, shape, transform))
                return false;
            mCached.reset();
            return true;
        }

        bool removeObject(std::size_t id);

        const RecastMesh& getMesh();

    private:
        RecastMeshManager mImpl;
        boost::optional<RecastMesh> mCached;
    };

    using NavMeshPtr = std::shared_ptr<dtNavMesh>;
    using NavMeshConstPtr = std::shared_ptr<const dtNavMesh>;

    class NavMeshManager
    {
    public:
        template <class T>
        bool addObject(std::size_t id, const T& shape, const btTransform& transform)
        {
            return mRecastMeshManager.addObject(id, shape, transform);
        }

        bool removeObject(std::size_t id);

        NavMeshConstPtr getNavMesh(const osg::Vec3f& agentHalfExtents);

    private:
        RecastMeshManagerCache mRecastMeshManager;
    };

    class NavMeshManagerCache
    {
    public:
        template <class T>
        bool addObject(std::size_t id, const T& shape, const btTransform& transform)
        {
            if (!mImpl.addObject(id, shape, transform))
                return false;
            for (auto& v : mCache)
                v.second.mInvalid = true;
            return true;
        }

        bool removeObject(std::size_t id);

        void reset(const osg::Vec3f& agentHalfExtents);

        void update(const osg::Vec3f& agentHalfExtents);

        NavMeshConstPtr getNavMesh(const osg::Vec3f& agentHalfExtents);

    private:
        struct CacheItem
        {
            NavMeshConstPtr mValue;
            bool mInvalid;
        };

        NavMeshManager mImpl;
        std::map<osg::Vec3f, CacheItem> mCache;
    };

    class Navigator
    {
    public:
        void addAgent(const osg::Vec3f& agentHalfExtents);

        void removeAgent(const osg::Vec3f& agentHalfExtents);

        template <class T>
        bool addObject(std::size_t id, const T& shape, const btTransform& transform)
        {
            return mNavMeshManager.addObject(id, shape, transform);
        }

        bool removeObject(std::size_t id);

        void update();

        std::vector<osg::Vec3f> findPath(const osg::Vec3f& agentHalfExtents,
                                         const osg::Vec3f& start, const osg::Vec3f& end);

    private:
        NavMeshManagerCache mNavMeshManager;
        std::map<osg::Vec3f, std::size_t> mAgents;
    };
}
