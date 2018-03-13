#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_SETTINGSUTILS_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_SETTINGSUTILS_H

#include "settings.hpp"

#include <osg/Vec3f>

namespace DetourNavigator
{
    inline float getHeight(const osg::Vec3f& agentHalfExtents, const Settings& settings)
    {
        return 2.0f * agentHalfExtents.z() * settings.mRecastScaleFactor;
    }

    inline float getMaxClimb(const Settings& settings)
    {
        return settings.mMaxClimb * settings.mRecastScaleFactor;
    }

    inline float getRadius(const osg::Vec3f& agentHalfExtents, const Settings& settings)
    {
        return agentHalfExtents.x() * settings.mRecastScaleFactor;
    }

    inline osg::Vec3f toNavMeshCoordinates(osg::Vec3f position, const Settings& settings)
    {
        std::swap(position.y(), position.z());
        return position * settings.mRecastScaleFactor;
    }

    inline osg::Vec3f fromNavMeshCoordinates(osg::Vec3f position, const Settings& settings)
    {
        const auto factor = 1.0f / settings.mRecastScaleFactor;
        position *= factor;
        std::swap(position.y(), position.z());
        return position;
    }
}

#endif
