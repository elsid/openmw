#include "navmeshmanager.hpp"
#include "debug.hpp"
#include "dtstatus.hpp"
#include "exceptions.hpp"
#include "findsmoothpath.hpp"
#include "gettilespositions.hpp"
#include "makenavmesh.hpp"
#include "navmeshcacheitem.hpp"
#include "obstacleavoidancetype.hpp"
#include "queryfiltertype.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"
#include "sharednavmesh.hpp"

#include <DetourCommon.h>
#include <DetourCrowd.h>
#include <DetourNavMesh.h>

#include <BulletCollision/CollisionShapes/btConcaveShape.h>

#include <iostream>

namespace
{
    using DetourNavigator::ChangeType;
    using DetourNavigator::Flags;
    using DetourNavigator::Settings;
    using DetourNavigator::QueryFilterType;

    ChangeType addChangeType(const ChangeType current, const ChangeType add)
    {
        return current == add ? current : ChangeType::mixed;
    }

    std::shared_ptr<dtCrowd> makeCrowd(dtNavMesh* navMesh, const Settings& settings)
    {
        using namespace DetourNavigator;

        const auto crowd = std::make_shared<dtCrowd>();

        crowd->init(settings.mMaxAgents, settings.mMaxAgentRadius, navMesh);

        dtObstacleAvoidanceParams params = *crowd->getObstacleAvoidanceParams(0);

        params.velBias = 1.0f;
        params.adaptiveDivs = 5;
        params.adaptiveRings = 2;
        params.adaptiveDepth = 1;
        crowd->setObstacleAvoidanceParams(ObstacleAvoidanceType_low, &params);

        params.velBias = 1.0f;
        params.adaptiveDivs = 5;
        params.adaptiveRings = 2;
        params.adaptiveDepth = 2;
        crowd->setObstacleAvoidanceParams(ObstacleAvoidanceType_medium, &params);

        params.velBias = 1.0f;
        params.adaptiveDivs = 7;
        params.adaptiveRings = 2;
        params.adaptiveDepth = 3;
        crowd->setObstacleAvoidanceParams(ObstacleAvoidanceType_high, &params);

        params.velBias = 1.0f;
        params.adaptiveDivs = 7;
        params.adaptiveRings = 3;
        params.adaptiveDepth = 3;
        crowd->setObstacleAvoidanceParams(ObstacleAvoidanceType_veryHigh, &params);

        crowd->getEditableFilter(QueryFilterType_walkFlag)->setIncludeFlags(Flag_walk);
        crowd->getEditableFilter(QueryFilterType_swimFlag)->setIncludeFlags(Flag_swim);
        crowd->getEditableFilter(QueryFilterType_walkAndSwimFlags)->setIncludeFlags(Flag_walk | Flag_swim);

        return crowd;
    }

    struct Poly
    {
        dtPolyRef mRef;
        osg::Vec3f mPosition;
    };

    Poly findNearestPoly(const dtNavMesh& navMesh, osg::Vec3f halfExtents, osg::Vec3f position,
            const Flags includeFlags, const Settings& settings)
    {
        dtNavMeshQuery navMeshQuery;
        OPENMW_CHECK_DT_STATUS(navMeshQuery.init(&navMesh, settings.mMaxNavMeshQueryNodes));

        dtQueryFilter queryFilter;
        queryFilter.setIncludeFlags(includeFlags);

        Poly result;
        OPENMW_CHECK_DT_STATUS(navMeshQuery.findNearestPoly(position.ptr(), halfExtents.ptr(), &queryFilter,
            &result.mRef, result.mPosition.ptr()));

        return result;
    }

    QueryFilterType getQueryFilterType(const Flags flags)
    {
        using namespace DetourNavigator;

        switch (flags)
        {
            case Flag_walk:
                return QueryFilterType_walkFlag;
            case Flag_swim:
                return QueryFilterType_swimFlag;
            case Flag_walk | Flag_swim:
                return QueryFilterType_walkAndSwimFlags;
        }

        return QueryFilterType_allFlags;
    }

    osg::Vec3f getCrowdAgentPosition(const dtCrowdAgent& agent)
    {
        osg::Vec3f result;
        dtVcopy(result.ptr(), agent.npos);
        return result;
    }

    osg::Vec3f getCrowdAgentTarget(const dtCrowdAgent& agent)
    {
        osg::Vec3f result;
        dtVcopy(result.ptr(), agent.targetPos);
        return result;
    }
}

namespace DetourNavigator
{
    NavMeshManager::NavMeshManager(const Settings& settings)
        : mSettings(settings)
        , mRecastMeshManager(settings)
        , mAsyncNavMeshUpdater(settings, mRecastMeshManager)
    {}

    bool NavMeshManager::addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                                   const AreaType areaType)
    {
        if (!mRecastMeshManager.addObject(id, shape, transform, areaType))
            return false;
        addChangedTiles(shape, transform, ChangeType::add);
        return true;
    }

    bool NavMeshManager::updateObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        if (!mRecastMeshManager.updateObject(id, shape, transform, areaType))
            return false;
        addChangedTiles(shape, transform, ChangeType::mixed);
        return true;
    }

    bool NavMeshManager::removeObject(std::size_t id)
    {
        const auto object = mRecastMeshManager.removeObject(id);
        if (!object)
            return false;
        addChangedTiles(*object->mShape, object->mTransform, ChangeType::remove);
        return true;
    }

    void NavMeshManager::addAgent(const std::size_t id, const osg::Vec3f& position, const osg::Vec3f& agentHalfExtents,
            const dtCrowdAgentParams& crowdParams)
    {
        if (mCrowdAgents.count(id))
            return;
        const auto localPosition = toNavMeshCoordinates(position, mSettings);
        auto cached = mCache.find(agentHalfExtents);
        if (cached != mCache.end())
        {
            const auto crowdId = cached->second->mValue.lock().crowd().addAgent(localPosition.ptr(), &crowdParams);
            if (crowdId == -1)
                throw NavigatorException("Failed to add crowd agent");
            mCrowdAgents.insert(std::make_pair(id, CrowdAgent {crowdId, agentHalfExtents}));
            return;
        }
        auto navMesh = makeEmptyNavMesh(mSettings);
        auto crowd = makeCrowd(navMesh.get(), mSettings);
        const auto crowdId = crowd->addAgent(localPosition.ptr(), &crowdParams);
        if (crowdId == -1)
            throw NavigatorException("Failed to add crowd agent");
        mCrowdAgents.insert(std::make_pair(id, CrowdAgent {crowdId, agentHalfExtents}));
        mCache.insert(std::make_pair(agentHalfExtents,
            std::make_shared<NavMeshCacheItem>(std::move(navMesh), std::move(crowd), ++mGenerationCounter)));
        log("cache add for agent=", agentHalfExtents);
    }

    void NavMeshManager::removeAgent(const std::size_t id)
    {
        auto agent = mCrowdAgents.find(id);
        if (agent == mCrowdAgents.end())
            return;
        auto cached = mCache.find(agent->second.mAgentHalfExtents);
        if (cached != mCache.end())
            cached->second->mValue.lock().crowd().removeAgent(agent->second.mCrowdId);
        mCrowdAgents.erase(agent);
    }

    void NavMeshManager::updateAgent(const std::size_t id, const osg::Vec3f& position, const float speed)
    {
        const auto crowdAgent = mCrowdAgents.find(id);
        if (crowdAgent == mCrowdAgents.end())
            return;
        const auto localPosition = toNavMeshCoordinates(position, mSettings);
        const auto navMesh = getNavMesh(crowdAgent->second.mAgentHalfExtents).lock();
        const auto agent = navMesh.crowd().getEditableAgent(crowdAgent->second.mCrowdId);
        dtVcopy(agent->npos, localPosition.ptr());
        agent->params.maxSpeed = speed * mSettings.mRecastScaleFactor;
        agent->params.maxAcceleration = agent->params.maxSpeed;
    }

    void NavMeshManager::updateAgentId(const std::size_t id, const std::size_t newId)
    {
        if (id == newId)
            return;
        const auto crowdAgent = mCrowdAgents.find(id);
        if (crowdAgent == mCrowdAgents.end())
            return;
        mCrowdAgents.insert(std::make_pair(newId, crowdAgent->second));
        mCrowdAgents.erase(crowdAgent);
    }

    void NavMeshManager::reset(const osg::Vec3f& agentHalfExtents)
    {
        mCache.erase(agentHalfExtents);
    }

    void NavMeshManager::update(const osg::Vec3f& playerPosition, const osg::Vec3f& agentHalfExtents)
    {
        const auto playerTile = getTilePosition(toNavMeshCoordinates(playerPosition, mSettings), mSettings);
        if (mLastRecastMeshManagerRevision >= mRecastMeshManager.getRevision() && mPlayerTile
                && *mPlayerTile == playerTile)
            return;
        mLastRecastMeshManagerRevision = mRecastMeshManager.getRevision();
        mPlayerTile = playerTile;
        std::map<TilePosition, ChangeType> tilesToPost;
        const auto cached = getCached(agentHalfExtents);
        const auto changedTiles = mChangedTiles.find(agentHalfExtents);
        {
            const auto locked = cached->mValue.lock();
            if (changedTiles != mChangedTiles.end())
            {
                for (const auto& tile : changedTiles->second)
                    if (locked->getTileAt(tile.first.x(), tile.first.y(), 0))
                    {
                        auto tileToPost = tilesToPost.find(tile.first);
                        if (tileToPost == tilesToPost.end())
                            tilesToPost.insert(tile);
                        else
                            tileToPost->second = addChangeType(tileToPost->second, tile.second);
                    }
                for (const auto& tile : tilesToPost)
                    changedTiles->second.erase(tile.first);
                if (changedTiles->second.empty())
                    mChangedTiles.erase(changedTiles);
            }
            const auto maxTiles = locked->getParams()->maxTiles;
            mRecastMeshManager.forEachTilePosition([&] (const TilePosition& tile)
            {
                if (tilesToPost.count(tile))
                    return;
                const auto shouldAdd = shouldAddTile(tile, playerTile, maxTiles);
                const auto presentInNavMesh = bool(locked->getTileAt(tile.x(), tile.y(), 0));
                if (shouldAdd && !presentInNavMesh)
                    tilesToPost.insert(std::make_pair(tile, ChangeType::add));
                else if (!shouldAdd && presentInNavMesh)
                    tilesToPost.insert(std::make_pair(tile, ChangeType::mixed));
            });
        }
        mAsyncNavMeshUpdater.post(agentHalfExtents, cached, playerTile, tilesToPost);
        log("cache update posted for agent=", agentHalfExtents,
            " playerTile=", *mPlayerTile,
            " recastMeshManagerRevision=", mLastRecastMeshManagerRevision,
            " changedTiles=", changedTiles->second.size());
    }

    void NavMeshManager::updateCrowd(const float duration)
    {
        for (const auto& cached : mCache)
            cached.second->mValue.lock().crowd().update(duration, nullptr);
    }

    bool NavMeshManager::updateAgentTarget(const std::size_t id, osg::Vec3f position, const Flags includeFlags)
    {
        const auto crowdAgent = mCrowdAgents.find(id);
        if (crowdAgent == mCrowdAgents.end())
            return false;
        const auto localPosition = toNavMeshCoordinates(position, mSettings);
        const auto localHalfExtents = toNavMeshCoordinates(crowdAgent->second.mAgentHalfExtents, mSettings);
        bool result = false;
        const auto agentId = crowdAgent->second.mCrowdId;
        for (const auto& cached : mCache)
        {
            const auto navMesh = cached.second->mValue.lock();
            auto& crowd = navMesh.crowd();
            const auto agent = crowd.getAgent(agentId);
            const auto target = getCrowdAgentTarget(*agent);
            if (agent->targetState != DT_CROWDAGENT_TARGET_NONE && target == localPosition)
                continue;
            const auto poly = findNearestPoly(*navMesh, localHalfExtents, localPosition, includeFlags, mSettings);
            crowd.getEditableAgent(crowdAgent->second.mCrowdId)->params.queryFilterType = getQueryFilterType(includeFlags);
            crowd.requestMoveTarget(crowdAgent->second.mCrowdId, poly.mRef, localPosition.ptr());
        }
        return result;
    }

    osg::Vec3f NavMeshManager::getAgentTarget(const std::size_t id) const
    {
        const auto crowdAgent = mCrowdAgents.find(id);
        if (crowdAgent == mCrowdAgents.end())
            throw NavigatorException("Agent not found, id=" + std::to_string(id));
        osg::Vec3f result;
        {
            const auto navMesh = getNavMesh(crowdAgent->second.mAgentHalfExtents).lock();
            result = getCrowdAgentTarget(*navMesh.crowd().getAgent(crowdAgent->second.mCrowdId));
        }
        return fromNavMeshCoordinates(result, mSettings);
    }

    osg::Vec3f NavMeshManager::getAgentPosition(const std::size_t id) const
    {
        const auto crowdAgent = mCrowdAgents.find(id);
        if (crowdAgent == mCrowdAgents.end())
            throw NavigatorException("Agent not found, id=" + std::to_string(id));
        osg::Vec3f result;
        {
            const auto navMesh = getNavMesh(crowdAgent->second.mAgentHalfExtents).lock();
            result = getCrowdAgentPosition(*navMesh.crowd().getAgent(crowdAgent->second.mCrowdId));
        }
        return fromNavMeshCoordinates(result, mSettings);
    }

    SharedNavMesh NavMeshManager::getNavMesh(const osg::Vec3f& agentHalfExtents) const
    {
        return getCached(agentHalfExtents)->mValue;
    }

    std::map<osg::Vec3f, std::shared_ptr<NavMeshCacheItem>> NavMeshManager::getNavMeshes() const
    {
        return mCache;
    }

    void NavMeshManager::wait()
    {
        mAsyncNavMeshUpdater.wait();
    }

    void NavMeshManager::addChangedTiles(const btCollisionShape& shape, const btTransform& transform,
                                         const ChangeType changeType)
    {
        getTilesPositions(shape, transform, mSettings, [&] (const TilePosition& v) {
            for (const auto& cached : mCache)
                if (cached.second)
                {
                    auto& tiles = mChangedTiles[cached.first];
                    auto tile = tiles.find(v);
                    if (tile == tiles.end())
                        tiles.insert(std::make_pair(v, changeType));
                    else
                        tile->second = addChangeType(tile->second, changeType);
                }
        });
    }

    std::shared_ptr<NavMeshCacheItem> NavMeshManager::getCached(const osg::Vec3f& agentHalfExtents) const
    {
        const auto cached = mCache.find(agentHalfExtents);
        if (cached == mCache.end())
        {
            std::ostringstream stream;
            stream << "Agent with half extents is not found: " << agentHalfExtents;
            throw InvalidArgument(stream.str());
        }
        return cached->second;
    }
}
