#include "navigator.hpp"
#include "debug.hpp"
#include "settingsutils.hpp"

namespace DetourNavigator
{
    Navigator::Navigator(const Settings& settings)
        : mSettings(settings)
        , mNavMeshManager(mSettings)
    {
    }

    void Navigator::addAgent(const osg::Vec3f& agentHalfExtents)
    {
        ++mAgents[agentHalfExtents];
    }

    void Navigator::removeAgent(const osg::Vec3f& agentHalfExtents)
    {
        const auto it = mAgents.find(agentHalfExtents);
        if (it == mAgents.end() || --it->second)
            return;
        mAgents.erase(it);
        mNavMeshManager.reset(agentHalfExtents);
    }

    bool Navigator::removeObject(std::size_t id)
    {
        return mNavMeshManager.removeObject(id);
    }

    void Navigator::update()
    {
        for (const auto& v : mAgents)
            mNavMeshManager.update(v.first);
    }
}
