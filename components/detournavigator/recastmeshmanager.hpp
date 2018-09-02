#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHMANAGER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHMANAGER_H

#include "recastmeshbuilder.hpp"

#include <LinearMath/btTransform.h>

#include <boost/optional.hpp>

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
        };

        RecastMeshManager(const Settings& settings);

        bool addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform);

        boost::optional<Object> removeObject(std::size_t id);

        std::shared_ptr<RecastMesh> getMesh();

    private:
        const Settings& mSettings;
        bool mShouldRebuild = false;
        RecastMeshBuilder mMeshBuilder;
        std::unordered_map<std::size_t, Object> mObjects;

        void rebuild();
    };
}

#endif
