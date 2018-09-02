#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H

#include "recastmesh.hpp"

class btConcaveShape;
class btHeightfieldTerrainShape;
class btTransform;
class btTriangleCallback;
class btVector3;

namespace DetourNavigator
{
    class RecastMeshBuilder
    {
    public:
        RecastMeshBuilder(const Settings& settings);

        void addObject(const btConcaveShape& shape, const btTransform& transform);

        void addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform);

        std::shared_ptr<RecastMesh> create() const;

    private:
        const Settings* mSettings;
        std::vector<int> mIndices;
        std::vector<float> mVertices;

        void addObject(const btConcaveShape& shape, btTriangleCallback&& callback);

        void addVertex(const btVector3& worldPosition);
    };
}

#endif
