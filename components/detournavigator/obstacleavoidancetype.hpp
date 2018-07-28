#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_OBSTACLEAVOIDANCETYPE_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_OBSTACLEAVOIDANCETYPE_H

namespace DetourNavigator
{
    enum ObstacleAvoidanceType : unsigned char
    {
        ObstacleAvoidanceType_low = 0,
        ObstacleAvoidanceType_medium = 1,
        ObstacleAvoidanceType_high = 2,
        ObstacleAvoidanceType_veryHigh = 3,
    };
}

#endif
