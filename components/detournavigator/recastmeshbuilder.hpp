#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H

#include "recastmesh.hpp"
#include "tilebounds.hpp"

#include <LinearMath/btTransform.h>

#include <array>

class btBoxShape;
class btCollisionShape;
class btCompoundShape;
class btConcaveShape;
class btHeightfieldTerrainShape;
class btTriangleCallback;

namespace DetourNavigator
{
    struct Settings;

    class RecastMeshBuilder
    {
    public:
        RecastMeshBuilder(const Settings& settings, const TileBounds& bounds);

        void addObject(const btCollisionShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btCompoundShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btConcaveShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform, const AreaType areaType);

        void addObject(const btBoxShape& shape, const btTransform& transform, const AreaType areaType);

        void addWater(const int mCellSize, const btTransform& transform);

        std::shared_ptr<RecastMesh> create(std::size_t generation, std::size_t revision);

        void reset();

    private:
        template <std::size_t N>
        struct Shape
        {
            std::size_t mIndex;
            AreaType mAreaType;
            std::array<btVector3, N> mVertices;
        };

        static constexpr std::size_t sBoxVerticesNum = 8;
        static constexpr std::size_t sBoxIndicesNum = 36;
        static constexpr std::size_t sBoxTrianglesNum = 12;
        static constexpr std::size_t sCoordinatesNum = 3;
        static constexpr std::size_t sTriangleVerticesNum = 3;

        using Box = Shape<sBoxVerticesNum>;
        using Triangle = Shape<sTriangleVerticesNum>;

        std::reference_wrapper<const Settings> mSettings;
        TileBounds mBounds;
        std::vector<Triangle> mTriangles;
        std::vector<Box> mBoxes;
        std::vector<RecastMesh::Water> mWater;

        void addObject(const btConcaveShape& shape, const btTransform& transform, btTriangleCallback&& callback);

        void addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform, btTriangleCallback&& callback);

        void addTriangleVertex(const btVector3& worldPosition);

        void addVertex(const btVector3& worldPosition);
    };
}

#endif
