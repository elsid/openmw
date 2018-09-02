#include "navigator.hpp"
#include "debug.hpp"
#include "settingsutils.hpp"

#include <Recast.h>

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
        mNavMeshManager.addAgent(agentHalfExtents);
    }

    void Navigator::removeAgent(const osg::Vec3f& agentHalfExtents)
    {
        const auto it = mAgents.find(agentHalfExtents);
        if (it == mAgents.end() || --it->second)
            return;
        mAgents.erase(it);
        mNavMeshManager.reset(agentHalfExtents);
    }

    bool Navigator::addObject(std::size_t id, const ObjectShapes& shapes, const btTransform& transform)
    {
        bool result = mNavMeshManager.addObject(id, shapes.mShape, transform, RC_WALKABLE_AREA);
        if (shapes.mAvoid)
        {
            const auto avoidId = reinterpret_cast<std::size_t>(shapes.mAvoid);
            if (mNavMeshManager.addObject(avoidId, *shapes.mAvoid, transform, RC_NULL_AREA))
            {
                updateAvoidShapeId(id, avoidId);
                result = true;
            }
        }
        return result;
    }

    bool Navigator::updateObject(std::size_t id, const ObjectShapes& shapes, const btTransform& transform)
    {
        bool result = mNavMeshManager.updateObject(id, shapes.mShape, transform, RC_WALKABLE_AREA);
        if (shapes.mAvoid)
        {
            const auto avoidId = reinterpret_cast<std::size_t>(shapes.mAvoid);
            if (mNavMeshManager.updateObject(avoidId, *shapes.mAvoid, transform, RC_NULL_AREA))
            {
                updateAvoidShapeId(id, avoidId);
                result = true;
            }
        }
        return result;
    }

    bool Navigator::removeObject(std::size_t id)
    {
        bool result = mNavMeshManager.removeObject(id);
        const auto avoid = mAvoidIds.find(id);
        if (avoid == mAvoidIds.end())
            return result;
        return mNavMeshManager.removeObject(avoid->second) || result;
    }

    void Navigator::update(const osg::Vec3f& playerPosition)
    {
        for (const auto& v : mAgents)
            mNavMeshManager.update(playerPosition, v.first);
    }

    std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> Navigator::getNavMeshes() const
    {
        return mNavMeshManager.getNavMeshes();
    }

    const Settings& Navigator::getSettings() const
    {
        return mSettings;
    }

    void Navigator::wait()
    {
        mNavMeshManager.wait();
    }

    void Navigator::updateAvoidShapeId(std::size_t id, std::size_t avoidId)
    {
        auto inserted = mAvoidIds.insert(std::make_pair(id, avoidId));
        if (!inserted.second)
        {
            mNavMeshManager.removeObject(inserted.first->second);
            inserted.second = avoidId;
        }
    }
}
