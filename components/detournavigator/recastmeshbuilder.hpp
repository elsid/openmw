#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H

#include "recastmesh.hpp"
#include "tilebounds.hpp"

#include <LinearMath/btTransform.h>

#include <osg/Vec2f>
#include <osg/Vec2i>

class btBoxShape;
class btCollisionShape;
class btCompoundShape;
class btConcaveShape;
class btHeightfieldTerrainShape;
class btTriangleCallback;

namespace DetourNavigator
{
    class RecastMeshBuilder
    {
    public:
        RecastMeshBuilder(const Settings& settings, const TileBounds& bounds);

        bool addObject(const btCollisionShape& shape, const btTransform& transform, const AreaType areaType);

        bool addObject(const btCompoundShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btConcaveShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btBoxShape& shape, const btTransform& transform, const AreaType areaType);

        void addWater(const int mCellSize, const btScalar level, const btTransform& transform);

        std::shared_ptr<RecastMesh> create() const;

        void reset();

    private:
        struct Water
        {
            int mHalfCellSize;
            btScalar mLevel;
            btTransform mInversedTransform;
        };

        const Settings& mSettings;
        TileBounds mBounds;
        std::vector<int> mIndices;
        std::vector<float> mVertices;
        std::vector<AreaType> mAreaTypes;
        std::vector<Water> mWater;

        void addObject(const btConcaveShape& shape, const btTransform& transform, btTriangleCallback&& callback);

        void addTriangleVertex(const btVector3& worldPosition);

        void addVertex(const btVector3& worldPosition);

        bool isUnderwater(const btVector3* triangle, const btTransform& transform) const;

        bool isUnderwater(const btBoxShape& box, const btTransform& transform) const;

        bool isUnderwaterWater(const btVector3& position) const;
    };
}

#endif
