#include "recastmesh.hpp"
#include "chunkytrimesh.hpp"
#include "settings.hpp"

#include <Recast.h>

namespace DetourNavigator
{
    RecastMesh::RecastMesh(std::vector<int> indices, std::vector<float> vertices,
                           std::vector<unsigned char> flags, const Settings& settings)
        : mIndices(std::move(indices))
        , mVertices(std::move(vertices))
        , mFlags(std::move(flags))
        , mSettings(settings)
        , mChunkyTriMesh(new ChunkyTriMesh(mVertices, mIndices, mFlags, mSettings.mTrianglesPerChunk))
    {
        if (getTrianglesCount() != mFlags.size())
            throw std::invalid_argument("number of flags doesn't match number of triangles: triangles="
                                        + std::to_string(getTrianglesCount()) + ", flags="
                                        + std::to_string(mFlags.size()));
        if (getVerticesCount())
            rcCalcBounds(mVertices.data(), int(getVerticesCount()), mBoundsMin.ptr(), mBoundsMax.ptr());
    }
}
