#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHMANAGER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHMANAGER_H

#include "recastmeshbuilder.hpp"

#include <LinearMath/btTransform.h>

#include <osg/Vec2i>

#include <boost/optional.hpp>

#include <map>
#include <unordered_map>

class btCollisionShape;

namespace DetourNavigator
{
    class RecastMeshManager
    {
    public:
        struct Object
        {
            const btCollisionShape* mShape;
            btTransform mTransform;
            AreaType mAreaType;
        };

        struct Water
        {
            int mCellSize;
            btScalar mLevel;
            btTransform mTransform;
        };

        RecastMeshManager(const Settings& settings, const TileBounds& bounds);

        bool addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                       const AreaType areaType);

        bool updateObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                          const AreaType areaType);

        bool addWater(const osg::Vec2i& cellPosition, const int cellSize, const btScalar level,
                      const btTransform& transform);

        boost::optional<Water> removeWater(const osg::Vec2i& cellPosition);

        boost::optional<Object> removeObject(std::size_t id);

        std::shared_ptr<RecastMesh> getMesh();

        bool isEmpty() const;

    private:
        bool mShouldRebuild = false;
        RecastMeshBuilder mMeshBuilder;
        std::unordered_map<std::size_t, Object> mObjects;
        std::map<osg::Vec2i, Water> mWater;

        void rebuild();
    };
}

#endif
