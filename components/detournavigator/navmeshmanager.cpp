#include "navmeshmanager.hpp"
#include "debug.hpp"
#include "exceptions.hpp"
#include "gettilespositions.hpp"
#include "makenavmesh.hpp"
#include "navmeshcacheitem.hpp"
#include "settings.hpp"
#include "sharednavmesh.hpp"

#include <DetourNavMesh.h>

#include <BulletCollision/CollisionShapes/btConcaveShape.h>

#include <iostream>

namespace
{
    using DetourNavigator::ChangeType;

    ChangeType addChangeType(const ChangeType current, const ChangeType add)
    {
        return current == add ? current : ChangeType::mixed;
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
                                   const unsigned char flags)
    {
        if (!mRecastMeshManager.addObject(id, shape, transform, flags))
            return false;
        addChangedTiles(shape, transform, ChangeType::add);
        return true;
    }

    bool NavMeshManager::updateObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                                      const unsigned char flags)
    {
        if (!mRecastMeshManager.updateObject(id, shape, transform, flags))
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

    void NavMeshManager::addAgent(const osg::Vec3f& agentHalfExtents)
    {
        auto cached = mCache.find(agentHalfExtents);
        if (cached != mCache.end())
            return;
        mCache.insert(std::make_pair(agentHalfExtents,
            std::make_shared<NavMeshCacheItem>(makeEmptyNavMesh(mSettings), ++mGenerationCounter)));
        log("cache add for agent=", agentHalfExtents);
    }

    void NavMeshManager::reset(const osg::Vec3f& agentHalfExtents)
    {
        mCache.erase(agentHalfExtents);
    }

    void NavMeshManager::update(osg::Vec3f playerPosition, const osg::Vec3f& agentHalfExtents)
    {
        playerPosition *= mSettings.mRecastScaleFactor;
        std::swap(playerPosition.y(), playerPosition.z());
        const auto playerTile = getTilePosition(playerPosition, mSettings);
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
