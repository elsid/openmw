#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESH_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESH_H

#include "area_type.hpp"

#include <memory>
#include <vector>

#include <osg/Vec3f>

namespace DetourNavigator
{
    class ChunkyTriMesh;
    struct Settings;

    class RecastMesh
    {
    public:
        RecastMesh(std::vector<int> indices, std::vector<float> vertices,
                   std::vector<AreaType> areaTypes, const Settings& settings);

        const std::vector<int>& getIndices() const
        {
            return mIndices;
        }

        const std::vector<float>& getVertices() const
        {
            return mVertices;
        }

        const std::vector<AreaType>& getAreaTypes() const
        {
            return mAreaTypes;
        }

        std::size_t getVerticesCount() const
        {
            return mVertices.size() / 3;
        }

        std::size_t getTrianglesCount() const
        {
            return mIndices.size() / 3;
        }

        const ChunkyTriMesh& getChunkyTriMesh() const
        {
            return *mChunkyTriMesh;
        }

        const osg::Vec3f& getBoundsMin() const
        {
            return mBoundsMin;
        }

        const osg::Vec3f& getBoundsMax() const
        {
            return mBoundsMax;
        }

    private:
        std::vector<int> mIndices;
        std::vector<float> mVertices;
        std::vector<AreaType> mAreaTypes;
        const Settings& mSettings;
        std::unique_ptr<ChunkyTriMesh> mChunkyTriMesh;
        osg::Vec3f mBoundsMin;
        osg::Vec3f mBoundsMax;
    };
}

#endif
