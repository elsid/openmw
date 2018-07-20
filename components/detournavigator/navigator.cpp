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
        if (shapes.mWater)
        {
            const auto waterId = reinterpret_cast<std::size_t>(shapes.mWater);
            const btTransform waterTransform(
                btMatrix3x3::getIdentity(),
                btVector3(transform.getOrigin().x(), transform.getOrigin().y(), 0)
            );
            if (mNavMeshManager.addObject(waterId, *shapes.mWater, waterTransform, AreaType_water))
            {
                updateWaterShapeId(id, waterId);
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
        if (shapes.mWater)
        {
            const auto waterId = reinterpret_cast<std::size_t>(shapes.mWater);
            const btTransform waterTransform(
                btMatrix3x3::getIdentity(),
                btVector3(transform.getOrigin().x(), transform.getOrigin().y(), 0)
            );
            if (mNavMeshManager.updateObject(waterId, *shapes.mWater, waterTransform, AreaType_water))
            {
                updateWaterShapeId(id, waterId);
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
