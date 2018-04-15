#include "recastmesh.hpp"
#include "chunkytrimesh.hpp"
#include "settings.hpp"

#include <Recast.h>

namespace DetourNavigator
{
    RecastMesh::RecastMesh(std::vector<int> indices, std::vector<float> vertices, const Settings& settings)
        : mIndices(std::move(indices))
        , mVertices(std::move(vertices))
        , mSettings(settings)
        , mChunkyTriMesh(new ChunkyTriMesh(mVertices, mIndices, mSettings.mTrianglesPerChunk))
    {
        if (getVerticesCount())
            rcCalcBounds(mVertices.data(), int(getVerticesCount()), mBoundsMin.ptr(), mBoundsMax.ptr());
    }
}
