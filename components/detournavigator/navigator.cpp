#include "navigator.hpp"
#include "debug.hpp"
#include "settingsutils.hpp"

#include <Recast.h>
#include <DetourCrowd.h>

namespace DetourNavigator
{
    Navigator::Navigator(const Settings& settings)
        : mSettings(settings)
        , mNavMeshManager(mSettings)
    {
    }

    void Navigator::addAgent(const std::size_t id, const osg::Vec3f& position, const osg::Vec3f& agentHalfExtents,
            const dtCrowdAgentParams& crowdParams)
    {
        ++mAgents[agentHalfExtents];
        mNavMeshManager.addAgent(id, position, agentHalfExtents, crowdParams);
    }

    void Navigator::removeAgent(const std::size_t id, const osg::Vec3f& agentHalfExtents)
    {
        mNavMeshManager.removeAgent(id);
        const auto it = mAgents.find(agentHalfExtents);
        if (it == mAgents.end() || --it->second)
            return;
        mAgents.erase(it);
        mNavMeshManager.reset(agentHalfExtents);
    }

    bool Navigator::addObject(std::size_t id, const ObjectShapes& shapes, const btTransform& transform)
    {
        bool result = mNavMeshManager.addObject(id, shapes.mShape, transform, AreaType_ground);
        if (shapes.mAvoid)
        {
            const auto avoidId = reinterpret_cast<std::size_t>(shapes.mAvoid);
            if (mNavMeshManager.addObject(avoidId, *shapes.mAvoid, transform, AreaType_null))
            {
                updateAvoidShapeId(id, avoidId);
                result = true;
            }
        }
        return result;
    }

    bool Navigator::updateObject(std::size_t id, const ObjectShapes& shapes, const btTransform& transform)
    {
        bool result = mNavMeshManager.updateObject(id, shapes.mShape, transform, AreaType_ground);
        if (shapes.mAvoid)
        {
            const auto avoidId = reinterpret_cast<std::size_t>(shapes.mAvoid);
            if (mNavMeshManager.updateObject(avoidId, *shapes.mAvoid, transform, AreaType_null))
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
        if (avoid != mAvoidIds.end())
            result = mNavMeshManager.removeObject(avoid->second) || result;
        const auto water = mWaterIds.find(id);
        if (water != mWaterIds.end())
            result = mNavMeshManager.removeObject(water->second) || result;
        return result;
    }

    bool Navigator::addWater(const osg::Vec2i& cellPosition, const int cellSize, const float level, const btTransform& transform)
    {
        return mNavMeshManager.addWater(cellPosition, cellSize, level, transform);
    }

    bool Navigator::removeWater(const osg::Vec2i& cellPosition)
    {
        return mNavMeshManager.removeWater(cellPosition);
    }

    void Navigator::update(const osg::Vec3f& playerPosition)
    {
        for (const auto& v : mAgents)
            mNavMeshManager.update(playerPosition, v.first);
    }

    void Navigator::updateCrowd(const float duration)
    {
        mNavMeshManager.updateCrowd(duration);
    }

    void Navigator::updateAgent(const std::size_t id, const osg::Vec3f& position, const float speed)
    {
        mNavMeshManager.updateAgent(id, position, speed);
    }

    void Navigator::updateAgentId(const std::size_t id, const std::size_t newId)
    {
        mNavMeshManager.updateAgentId(id, newId);
    }

    bool Navigator::updateAgentTarget(const std::size_t id, const osg::Vec3f& position, const Flags includeFlags)
    {
        return mNavMeshManager.updateAgentTarget(id, position, includeFlags);
    }

    osg::Vec3f Navigator::getAgentTarget(const std::size_t id) const
    {
        return mNavMeshManager.getAgentTarget(id);
    }

    osg::Vec3f Navigator::getAgentPosition(const std::size_t id) const
    {
        return mNavMeshManager.getAgentPosition(id);
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

    void Navigator::updateAvoidShapeId(const std::size_t id, const std::size_t avoidId)
    {
        updateId(id, avoidId, mWaterIds);
    }

    void Navigator::updateWaterShapeId(const std::size_t id, const std::size_t waterId)
    {
        updateId(id, waterId, mWaterIds);
    }

    void Navigator::updateId(const std::size_t id, const std::size_t updateId, std::unordered_map<std::size_t, std::size_t>& ids)
    {
        auto inserted = ids.insert(std::make_pair(id, updateId));
        if (!inserted.second)
        {
            mNavMeshManager.removeObject(inserted.first->second);
            inserted.second = updateId;
        }
    }
}
