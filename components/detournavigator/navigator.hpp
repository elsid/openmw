#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVIGATOR_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_NAVIGATOR_H

#include "findsmoothpath.hpp"
#include "navmeshmanager.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"

namespace DetourNavigator
{
    class Navigator
    {
    public:
        Navigator(const Settings& settings);

        void addAgent(const osg::Vec3f& agentHalfExtents);

        void removeAgent(const osg::Vec3f& agentHalfExtents);

        template <class T>
        bool addObject(std::size_t id, const T& shape, const btTransform& transform)
        {
            return mNavMeshManager.addObject(id, shape, transform);
        }

        bool removeObject(std::size_t id);

        void update(const osg::Vec3f& playerPosition);

        template <class OutputIterator>
        OutputIterator findPath(const osg::Vec3f& agentHalfExtents, const osg::Vec3f& start,
            const osg::Vec3f& end, OutputIterator out) const
        {
            const auto navMesh = mNavMeshManager.getNavMesh(agentHalfExtents);
            return findSmoothPath(*navMesh.lock(), toNavMeshCoordinates(agentHalfExtents, mSettings),
                toNavMeshCoordinates(start, mSettings), toNavMeshCoordinates(end, mSettings), mSettings, out);
        }

    private:
        Settings mSettings;
        NavMeshManager mNavMeshManager;
        std::map<osg::Vec3f, std::size_t> mAgents;
    };
}

#endif
