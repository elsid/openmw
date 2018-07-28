#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVIGATOR_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVIGATOR_H

#include "findsmoothpath.hpp"
#include "flags.hpp"
#include "navmeshmanager.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"
#include "water.hpp"

namespace DetourNavigator
{
    struct ObjectShapes
    {
        const btCollisionShape& mShape;
        const btCollisionShape* mAvoid;
        const btCollisionShape* mWater;

        ObjectShapes(const btCollisionShape& shape, const btCollisionShape* avoid = nullptr)
            : mShape(shape), mAvoid(avoid), mWater(nullptr)
        {}

        ObjectShapes(const btHeightfieldTerrainShape& shape, const Water& water)
            : mShape(shape), mAvoid(nullptr), mWater(&water.getImpl())
        {}
    };

    class Navigator
    {
    public:
        Navigator(const Settings& settings);

        void addAgent(const std::size_t id, const osg::Vec3f& position, const osg::Vec3f& agentHalfExtents,
                      const dtCrowdAgentParams& crowdParams);

        void removeAgent(const std::size_t id, const osg::Vec3f& agentHalfExtents);

        bool addObject(std::size_t id, const ObjectShapes& shapes, const btTransform& transform);

        bool updateObject(std::size_t id, const ObjectShapes& shapes, const btTransform& transform);

        bool removeObject(std::size_t id);

        void update(const osg::Vec3f& playerPosition);

        template <class OutputIterator>
        OutputIterator findPath(const osg::Vec3f& agentHalfExtents, const osg::Vec3f& start,
            const osg::Vec3f& end, const Flags includeFlags, OutputIterator out) const
        {
            const auto navMesh = mNavMeshManager.getNavMesh(agentHalfExtents);
            return findSmoothPath(*navMesh.lock(), toNavMeshCoordinates(agentHalfExtents, mSettings),
                toNavMeshCoordinates(start, mSettings), toNavMeshCoordinates(end, mSettings), includeFlags,
                mSettings, out);
        }

        std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> getNavMeshes() const;

        const Settings& getSettings() const;

        void updateCrowd(const float duration);

        void updateAgent(const std::size_t id, const osg::Vec3f& position, const float speed);

        void updateAgentId(const std::size_t id, const std::size_t newId);

        bool updateAgentTarget(const std::size_t id, const osg::Vec3f& position, const Flags includeFlags);

        osg::Vec3f getAgentTarget(const std::size_t id) const;

        osg::Vec3f getAgentPosition(const std::size_t id) const;

        void wait();

    private:
        Settings mSettings;
        NavMeshManager mNavMeshManager;
        std::map<osg::Vec3f, std::size_t> mAgents;
        std::unordered_map<std::size_t, std::size_t> mAvoidIds;
        std::unordered_map<std::size_t, std::size_t> mWaterIds;

        void updateAvoidShapeId(const std::size_t id, const std::size_t avoidId);
        void updateWaterShapeId(const std::size_t id, const std::size_t waterId);
        void updateId(const std::size_t id, const std::size_t waterId, std::unordered_map<std::size_t, std::size_t>& ids);
    };
}

#endif
