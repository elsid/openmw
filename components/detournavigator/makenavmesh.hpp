#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_MAKENAVMESH_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_MAKENAVMESH_H

#include "settings.hpp"
#include "navmeshcacheitem.hpp"
#include "tileposition.hpp"
#include "tilebounds.hpp"

#include <osg/Vec3f>

#include <memory>
#include <set>

class dtNavMesh;

namespace DetourNavigator
{
    class RecastMesh;
    class SharedNavMesh;
    struct Settings;

    using NavMeshPtr = std::shared_ptr<dtNavMesh>;

    enum class UpdateNavMeshStatus
    {
        ignore,
        removed,
        add,
        replaced
    };

    inline float getLength(const osg::Vec2i& value)
    {
        return std::sqrt(float(osg::square(value.x()) + osg::square(value.y())));
    }

    inline float getDistance(const TilePosition& lhs, const TilePosition& rhs)
    {
        return getLength(lhs - rhs);
    }

    inline bool shouldAddTile(const TilePosition& changedTile, const TilePosition& playerTile, int maxTiles)
    {
        const auto expectedTilesCount = std::ceil(osg::PI * osg::square(getDistance(changedTile, playerTile)));
        return expectedTilesCount * 3 <= maxTiles;
    }

    NavMeshPtr makeEmptyNavMesh(const Settings& settings);

    UpdateNavMeshStatus updateNavMesh(const osg::Vec3f& agentHalfExtents,
            const std::shared_ptr<RecastMesh>& recastMesh, const TilePosition& changedTile,
            const TilePosition& playerTile, const Settings& settings, NavMeshCacheItem& navMeshCacheItem);
}

#endif
