#include "makenavmesh.hpp"
#include "chunkytrimesh.hpp"
#include "dtstatus.hpp"
#include "exceptions.hpp"
#include "recastmesh.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"

#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <Recast.h>
#include <RecastAlloc.h>

#include <boost/optional.hpp>

namespace
{
    using namespace DetourNavigator;

    void initPolyMeshDetail(rcPolyMeshDetail& value)
    {
        value.meshes = nullptr;
        value.verts = nullptr;
        value.tris = nullptr;
    }

    struct PolyMeshDetailStackDeleter
    {
        void operator ()(rcPolyMeshDetail* value) const
        {
            rcFree(value->meshes);
            rcFree(value->verts);
            rcFree(value->tris);
        }
    };

    using PolyMeshDetailStackPtr = std::unique_ptr<rcPolyMeshDetail, PolyMeshDetailStackDeleter>;

    struct NavMeshDataValueDeleter
    {
        void operator ()(unsigned char* value) const
        {
            dtFree(value);
        }
    };

    using NavMeshDataValue = std::unique_ptr<unsigned char, NavMeshDataValueDeleter>;

    struct NavMeshData
    {
        NavMeshDataValue mValue;
        int mSize;

        NavMeshData() = default;

        NavMeshData(unsigned char* value, int size)
            : mValue(value)
            , mSize(size)
        {}
    };

    NavMeshData makeNavMeshTileData(const osg::Vec3f& agentHalfExtents, const RecastMesh& recastMesh,
                                        int tileX, int tileY, int tileSize, const osg::Vec3f& boundsMin,
                                        const osg::Vec3f& boundsMax, const Settings& settings)
    {
        rcContext context;
        rcConfig config;

        config.cs = settings.mCellSize;
        config.ch = settings.mCellHeight;
        config.walkableSlopeAngle = settings.mMaxSlope;
        config.walkableHeight = int(std::ceil(getHeight(agentHalfExtents, settings) / config.ch));
        config.walkableClimb = int(std::floor(getMaxClimb(settings) / config.ch));
        config.walkableRadius = int(std::ceil(getRadius(agentHalfExtents, settings) / config.cs));
        config.maxEdgeLen = int(std::round(settings.mMaxEdgeLen / config.cs));
        config.maxSimplificationError = settings.mMaxSimplificationError;
        config.minRegionArea = settings.mRegionMinSize * settings.mRegionMinSize;
        config.mergeRegionArea = settings.mRegionMergeSize * settings.mRegionMergeSize;
        config.maxVertsPerPoly = settings.mMaxVertsPerPoly;
        config.detailSampleDist = settings.mDetailSampleDist < 0.9f ? 0 : config.cs * settings.mDetailSampleDist;
        config.detailSampleMaxError = config.ch * settings.mDetailSampleMaxError;
        config.tileSize = tileSize;
        config.borderSize = config.walkableRadius + 3;
        config.width = config.tileSize + config.borderSize * 2;
        config.height = config.tileSize + config.borderSize * 2;
        rcVcopy(config.bmin, boundsMin.ptr());
        rcVcopy(config.bmax, boundsMax.ptr());
        config.bmin[0] -= config.borderSize * config.cs;
        config.bmin[2] -= config.borderSize * config.cs;
        config.bmax[0] += config.borderSize * config.cs;
        config.bmax[2] += config.borderSize * config.cs;

        rcHeightfield solid;
        OPENMW_CHECK_DT_RESULT(rcCreateHeightfield(nullptr, solid, config.width, config.height,
                config.bmin, config.bmax, config.cs, config.ch));

        {
            const auto& chunkyMesh = recastMesh.getChunkyTriMesh();
            std::vector<unsigned char> areas(chunkyMesh.getMaxTrisPerChunk(), 0);
            const osg::Vec2f tileBoundsMin(config.bmin[0], config.bmin[2]);
            const osg::Vec2f tileBoundsMax(config.bmax[0], config.bmax[2]);
            std::vector<std::size_t> cids;
            chunkyMesh.getChunksOverlappingRect(Rect {tileBoundsMin, tileBoundsMax}, std::back_inserter(cids));

            if (cids.empty())
                return NavMeshData();

            for (const auto cid : cids)
            {
                const auto chunk = chunkyMesh.getChunk(cid);

                std::fill(
                    areas.begin(),
                    std::min(areas.begin() + static_cast<std::ptrdiff_t>(chunk.mSize),
                    areas.end()),
                    0
                );

                rcMarkWalkableTriangles(
                    &context,
                    config.walkableSlopeAngle,
                    recastMesh.getVertices().data(),
                    int(recastMesh.getVerticesCount()),
                    chunk.mIndices,
                    static_cast<int>(chunk.mSize),
                    areas.data()
                );

                OPENMW_CHECK_DT_RESULT(rcRasterizeTriangles(
                    &context,
                    recastMesh.getVertices().data(),
                    int(recastMesh.getVerticesCount()),
                    chunk.mIndices,
                    areas.data(),
                    static_cast<int>(chunk.mSize),
                    solid,
                    config.walkableClimb
                ));
            }
        }

        rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, solid);
        rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, solid);
        rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, solid);

        rcPolyMesh polyMesh;
        rcPolyMeshDetail polyMeshDetail;
        initPolyMeshDetail(polyMeshDetail);
        const PolyMeshDetailStackPtr polyMeshDetailPtr(&polyMeshDetail);
        {
            rcCompactHeightfield compact;

            OPENMW_CHECK_DT_RESULT(rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb,
                                                             solid, compact));
            OPENMW_CHECK_DT_RESULT(rcErodeWalkableArea(&context, config.walkableRadius, compact));
            OPENMW_CHECK_DT_RESULT(rcBuildDistanceField(&context, compact));
            OPENMW_CHECK_DT_RESULT(rcBuildRegions(&context, compact, config.borderSize, config.minRegionArea,
                                                  config.mergeRegionArea));

            rcContourSet contourSet;
            OPENMW_CHECK_DT_RESULT(rcBuildContours(&context, compact, config.maxSimplificationError, config.maxEdgeLen,
                                                   contourSet));

            if (contourSet.nconts == 0)
                return NavMeshData();

            OPENMW_CHECK_DT_RESULT(rcBuildPolyMesh(&context, contourSet, config.maxVertsPerPoly, polyMesh));
            OPENMW_CHECK_DT_RESULT(rcBuildPolyMeshDetail(&context, polyMesh, compact, config.detailSampleDist,
                                                         config.detailSampleMaxError, polyMeshDetail));
        }

        for (int i = 0; i < polyMesh.npolys; ++i)
            if (polyMesh.areas[i] == RC_WALKABLE_AREA)
                polyMesh.flags[i] = 1;

        dtNavMeshCreateParams params;
        params.verts = polyMesh.verts;
        params.vertCount = polyMesh.nverts;
        params.polys = polyMesh.polys;
        params.polyAreas = polyMesh.areas;
        params.polyFlags = polyMesh.flags;
        params.polyCount = polyMesh.npolys;
        params.nvp = polyMesh.nvp;
        params.detailMeshes = polyMeshDetail.meshes;
        params.detailVerts = polyMeshDetail.verts;
        params.detailVertsCount = polyMeshDetail.nverts;
        params.detailTris = polyMeshDetail.tris;
        params.detailTriCount = polyMeshDetail.ntris;
        params.offMeshConVerts = nullptr;
        params.offMeshConRad = nullptr;
        params.offMeshConDir = nullptr;
        params.offMeshConAreas = nullptr;
        params.offMeshConFlags = nullptr;
        params.offMeshConUserID = nullptr;
        params.offMeshConCount = 0;
        params.walkableHeight = getHeight(agentHalfExtents, settings);
        params.walkableRadius = getRadius(agentHalfExtents, settings);
        params.walkableClimb = getMaxClimb(settings);
        rcVcopy(params.bmin, polyMesh.bmin);
        rcVcopy(params.bmax, polyMesh.bmax);
        params.cs = config.cs;
        params.ch = config.ch;
        params.buildBvTree = true;
        params.userId = 0;
        params.tileX = tileX;
        params.tileY = tileY;
        params.tileLayer = 0;

        unsigned char* navMeshData;
        int navMeshDataSize;
        OPENMW_CHECK_DT_RESULT(dtCreateNavMeshData(&params, &navMeshData, &navMeshDataSize));

        return NavMeshData(navMeshData, navMeshDataSize);
    }

    int nextPow2(int v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
    }

    NavMeshPtr makeNavMeshWithMultiTiles(const osg::Vec3f& agentHalfExtents, const RecastMesh& recastMesh,
                                            const Settings& settings)
    {
        osg::Vec3f boundsMin;
        osg::Vec3f boundsMax;
        rcCalcBounds(recastMesh.getVertices().data(), int(recastMesh.getVerticesCount()),
                        boundsMin.ptr(), boundsMax.ptr());

        int gridWidth;
        int gridHeight;
        rcCalcGridSize(boundsMin.ptr(), boundsMax.ptr(), settings.mCellSize, &gridWidth, &gridHeight);
        const auto tileSize = int(settings.mTileSize);
        const auto tileWidth = (gridWidth + tileSize - 1) / tileSize;
        const auto tileHeight = (gridHeight + tileSize - 1) / tileSize;
        const auto tileCellSize = settings.mTileSize * settings.mCellSize;

        // Max tiles and max polys affect how the tile IDs are caculated.
        // There are 22 bits available for identifying a tile and a polygon.
        const auto tileBits = std::min(int(std::log2(nextPow2(tileWidth * tileHeight))), 14);
        const auto polyBits = 22 - tileBits;
        const auto maxTiles = 1 << tileBits;
        const auto maxPolysPerTile = 1 << polyBits;

        dtNavMeshParams params;
        rcVcopy(params.orig, boundsMin.ptr());
        params.tileWidth = settings.mTileSize * settings.mCellSize;
        params.tileHeight = settings.mTileSize * settings.mCellSize;
        params.maxTiles = maxTiles;
        params.maxPolys = maxPolysPerTile;

        NavMeshPtr navMesh(dtAllocNavMesh(), &dtFreeNavMesh);
        OPENMW_CHECK_DT_STATUS(navMesh->init(&params));

        for (int y = 0; y < tileHeight; ++y)
        {
            for (int x = 0; x < tileWidth; ++x)
            {
                const osg::Vec3f tileBorderMin = boundsMin + osg::Vec3f(x * tileCellSize, 0, y * tileCellSize);
                const osg::Vec3f tileBorderMax(boundsMin.x() + (x + 1) * tileCellSize, boundsMax.y(),
                                                boundsMin.z() + (y + 1) * tileCellSize);

                auto navMeshData = makeNavMeshTileData(agentHalfExtents, recastMesh, x, y, tileSize,
                                                        tileBorderMin, tileBorderMax, settings);

                if (!navMeshData.mValue)
                    continue;

                OPENMW_CHECK_DT_STATUS(navMesh->addTile(navMeshData.mValue.get(), navMeshData.mSize,
                                                        DT_TILE_FREE_DATA, 0, 0));
                navMeshData.mValue.release();
            }
        }

        return navMesh;
    }
}

namespace DetourNavigator
{
    NavMeshPtr makeNavMesh(const osg::Vec3f& agentHalfExtents, const RecastMesh& recastMesh, const Settings& settings)
    {
        return makeNavMeshWithMultiTiles(agentHalfExtents, recastMesh, settings);
    }
}
