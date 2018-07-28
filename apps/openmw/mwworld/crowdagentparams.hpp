#ifndef OPENMW_MWWORLD_CROWDAGENTPARAMS_H
#define OPENMW_MWWORLD_CROWDAGENTPARAMS_H

#include <components/detournavigator/obstacleavoidancetype.hpp>
#include <components/detournavigator/queryfiltertype.hpp>
#include <components/detournavigator/settingsutils.hpp>

#include "class.hpp"
#include "ptr.hpp"

#include <DetourCrowd.h>

namespace MWWorld
{
    inline dtCrowdAgentParams makeCrowdAgentParams(const MWWorld::Ptr& actor, const osg::Vec3f& halfExtents,
            const DetourNavigator::Settings& settings)
    {
        using namespace DetourNavigator;

        dtCrowdAgentParams result;

        result.radius = getRadius(halfExtents, settings);
        result.height = getHeight(halfExtents, settings);
        result.maxSpeed = actor.getClass().getSpeed(actor) * settings.mRecastScaleFactor;
        result.maxAcceleration = result.maxSpeed;
        result.collisionQueryRange = result.radius * 50;
        result.pathOptimizationRange = result.radius * 100;
        result.separationWeight = result.radius * 2;
        result.updateFlags = DT_CROWD_ANTICIPATE_TURNS
            | DT_CROWD_OBSTACLE_AVOIDANCE
            | DT_CROWD_SEPARATION
            | DT_CROWD_OPTIMIZE_VIS
            | DT_CROWD_OPTIMIZE_TOPO;
        result.obstacleAvoidanceType = ObstacleAvoidanceType_veryHigh;
        result.queryFilterType = QueryFilterType_allFlags;
        result.userData = nullptr;

        return result;
    }
}

#endif
