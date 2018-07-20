#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_WATER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_WATER_H

#include <components/esm/loadland.hpp>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <array>

namespace DetourNavigator
{
    class Water
    {
    public:
        Water(const btScalar level, const btScalar scale)
            : mHeightfieldData {{level, level, level, level}}
            , mImpl(2, 2, mHeightfieldData.data(), 1, level, level, 2, PHY_FLOAT, false)
        {
            mImpl.setLocalScaling(btVector3(scale, scale, 1));
        }

        Water(const Water&) = delete;
        Water(Water&&) = delete;

        const btHeightfieldTerrainShape& getImpl() const
        {
            return mImpl;
        }

    private:
        std::array<btScalar, 4> mHeightfieldData;
        btHeightfieldTerrainShape mImpl;
    };
}

#endif
