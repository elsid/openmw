#include "navigator.hpp"

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <Recast.h>
#include <RecastAlloc.h>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btConcaveShape.h>

#include <osg/Vec3f>

#include <boost/optional.hpp>

#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace DetourNavigator
{
    namespace DetourTraits
    {
        static const float cellHeight = 0.2f;
        static const float cellSize = 0.2f;
        static const float detailSampleDist = 6.0f;
        static const float detailSampleMaxError = 1.0f;
        static const float maxClimb = 34.0f;
        static const float maxSimplificationError = 1.3f;
        static const float maxSlope = 49.0f;
        static const int maxEdgeLen = 12;
        static const int maxVertsPerPoly = 6;
        static const int regionMergeSize = 20;
        static const int regionMinSize = 8;

        // Scale all coordinates to change order of values to make rcCreateHeightfield work
        static const float invertedRecastScaleFactor = 75.0f;
        static const float recastScaleFactor = 1.0f / invertedRecastScaleFactor;
    }

    static std::ostream& operator <<(std::ostream& stream, const osg::Vec3f& value)
    {
        return stream << '(' << value.x() << ", " << value.y() << ", " << value.z() << ')';
    }

    namespace
    {
#define OPENMW_DT_STATUS(status) {status, #status}

        static const std::vector<std::pair<dtStatus, const char*>> dtStatuses {
            OPENMW_DT_STATUS(DT_FAILURE),
            OPENMW_DT_STATUS(DT_SUCCESS),
            OPENMW_DT_STATUS(DT_IN_PROGRESS),
            OPENMW_DT_STATUS(DT_WRONG_MAGIC),
            OPENMW_DT_STATUS(DT_WRONG_VERSION),
            OPENMW_DT_STATUS(DT_OUT_OF_MEMORY),
            OPENMW_DT_STATUS(DT_INVALID_PARAM),
            OPENMW_DT_STATUS(DT_BUFFER_TOO_SMALL),
            OPENMW_DT_STATUS(DT_OUT_OF_NODES),
            OPENMW_DT_STATUS(DT_PARTIAL_RESULT),
        };

#undef OPENMW_DT_STATUS

        struct WriteDtStatus
        {
            dtStatus status;
        };

        std::ostream& operator <<(std::ostream& stream, const WriteDtStatus& value)
        {
            for (const auto& status : dtStatuses)
                if (value.status & status.first)
                    stream << status.second << " ";
            return stream;
        }

        void checkDtStatus(dtStatus status, const char* call, int line)
        {
            if (!dtStatusSucceed(status))
            {
                std::ostringstream message;
                message << call << " failed with status=" << WriteDtStatus {status} << " at " __FILE__ ":" << line;
                throw NavigatorException(message.str());
            }
        }

#define OPENMW_CHECK_DT_STATUS(call) do { checkDtStatus((call), #call, __LINE__); } while (false)

        void checkDtResult(bool result, const char* call, int line)
        {
            if (!result)
            {
                std::ostringstream message;
                message << call << " failed at " __FILE__ ":" << line;
                throw NavigatorException(message.str());
            }
        }

#define OPENMW_CHECK_DT_RESULT(call) do { checkDtResult((call), #call, __LINE__); } while (false)


        float getHeight(const osg::Vec3f& agentHalfExtents)
        {
            return 2.0f * agentHalfExtents.z() * DetourTraits::recastScaleFactor;
        }

        float getMaxClimb()
        {
            return DetourTraits::maxClimb * DetourTraits::recastScaleFactor;
        }

        float getRadius(const osg::Vec3f& agentHalfExtents)
        {
            return agentHalfExtents.x() * DetourTraits::recastScaleFactor;
        }

        void initCompactHeightfield(rcCompactHeightfield& value)
        {
            value.cells = nullptr;
            value.spans = nullptr;
            value.dist = nullptr;
            value.areas = nullptr;
            value.borderSize = 0;
        }

        void freeCompactHeightfield(rcCompactHeightfield* value)
        {
            rcFree(value->cells);
            rcFree(value->spans);
            rcFree(value->dist);
            rcFree(value->areas);
        }

        using CompactHeightfieldStackPtr = std::unique_ptr<rcCompactHeightfield, decltype(&freeCompactHeightfield)>;

        void initContourSet(rcContourSet& value)
        {
            value.conts = nullptr;
            value.nconts = 0;
        }

        void freeContourSet(rcContourSet* value)
        {
            for (int i = 0; i < value->nconts; ++i)
            {
                rcFree(value->conts[i].verts);
                rcFree(value->conts[i].rverts);
            }
            rcFree(value->conts);
        }

        using ContourSetStackPtr = std::unique_ptr<rcContourSet, decltype(&freeContourSet)>;

        void initPolyMesh(rcPolyMesh& value)
        {
            value.verts = nullptr;
            value.polys = nullptr;
            value.regs = nullptr;
            value.flags = nullptr;
            value.areas = nullptr;
        }

        void freePolyMesh(rcPolyMesh* value)
        {
            rcFree(value->verts);
            rcFree(value->polys);
            rcFree(value->regs);
            rcFree(value->flags);
            rcFree(value->areas);
        }

        using PolyMeshStackPtr = std::unique_ptr<rcPolyMesh, decltype(&freePolyMesh)>;

        void initPolyMesh(rcPolyMeshDetail& value)
        {
            value.meshes = nullptr;
            value.verts = nullptr;
            value.tris = nullptr;
        }

        void freePolyMeshDetail(rcPolyMeshDetail* value)
        {
            rcFree(value->meshes);
            rcFree(value->verts);
            rcFree(value->tris);
        }

        using PolyMeshDetailStackPtr = std::unique_ptr<rcPolyMeshDetail, decltype(&freePolyMeshDetail)>;

        NavMeshPtr makeNavMesh(const osg::Vec3f& agentHalfExtents, const RecastMesh& recastMesh)
        {
            rcContext context;
            rcConfig config;

            config.tileSize = 0;
            config.cs = DetourTraits::cellSize;
            config.ch = DetourTraits::cellHeight;
            config.walkableSlopeAngle = DetourTraits::maxSlope;
            config.walkableHeight = int(std::ceil(getHeight(agentHalfExtents) / config.ch));
            config.walkableClimb = int(std::floor(getMaxClimb() / config.ch));
            config.walkableRadius = int(std::ceil(getRadius(agentHalfExtents) / config.cs));
            config.maxEdgeLen = int(std::round(DetourTraits::maxEdgeLen / config.cs));
            config.maxSimplificationError = DetourTraits::maxSimplificationError;
            config.minRegionArea = DetourTraits::regionMinSize * DetourTraits::regionMinSize;
            config.mergeRegionArea = DetourTraits::regionMergeSize * DetourTraits::regionMergeSize;
            config.maxVertsPerPoly = DetourTraits::maxVertsPerPoly;
            config.detailSampleDist = DetourTraits::detailSampleDist < 0.9f ? 0 : config.cs * DetourTraits::detailSampleDist;
            config.detailSampleMaxError = config.cs * DetourTraits::detailSampleMaxError;

            rcCalcBounds(recastMesh.getVertices().data(), int(recastMesh.getVerticesCount()), config.bmin, config.bmax);

            rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

            rcHeightfield solid;
            OPENMW_CHECK_DT_RESULT(rcCreateHeightfield(nullptr, solid, config.width, config.height, config.bmin, config.bmax,
                                                       config.cs, config.ch));

            std::vector<unsigned char> areas(recastMesh.getTrianglesCount(), 0);
            rcMarkWalkableTriangles(
                &context,
                config.walkableSlopeAngle,
                recastMesh.getVertices().data(),
                int(recastMesh.getVerticesCount()),
                recastMesh.getIndices().data(),
                int(recastMesh.getTrianglesCount()),
                areas.data()
            );

            OPENMW_CHECK_DT_RESULT(rcRasterizeTriangles(
                &context,
                recastMesh.getVertices().data(),
                int(recastMesh.getVerticesCount()),
                recastMesh.getIndices().data(),
                areas.data(),
                int(recastMesh.getTrianglesCount()),
                solid,
                config.walkableClimb
            ));

            rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, solid);
            rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, solid);
            rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, solid);

            rcPolyMesh polyMesh;
            initPolyMesh(polyMesh);
            const PolyMeshStackPtr polyMeshPtr(&polyMesh, &freePolyMesh);
            rcPolyMeshDetail polyMeshDetail;
            initPolyMesh(polyMeshDetail);
            const PolyMeshDetailStackPtr polyMeshDetailPtr(&polyMeshDetail, &freePolyMeshDetail);
            {
                rcCompactHeightfield compact;
                initCompactHeightfield(compact);
                const CompactHeightfieldStackPtr compatPtr(&compact, &freeCompactHeightfield);

                OPENMW_CHECK_DT_RESULT(rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb,
                                                                 solid, compact));
                OPENMW_CHECK_DT_RESULT(rcErodeWalkableArea(&context, config.walkableRadius, compact));
                OPENMW_CHECK_DT_RESULT(rcBuildDistanceField(&context, compact));
                OPENMW_CHECK_DT_RESULT(rcBuildRegions(&context, compact, 0, config.minRegionArea, config.mergeRegionArea));

                rcContourSet contourSet;
                initContourSet(contourSet);
                const ContourSetStackPtr contourSetPtr(&contourSet, &freeContourSet);
                OPENMW_CHECK_DT_RESULT(rcBuildContours(&context, compact, config.maxSimplificationError, config.maxEdgeLen,
                                                       contourSet));
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
            params.walkableHeight = getHeight(agentHalfExtents);
            params.walkableRadius = getRadius(agentHalfExtents);
            params.walkableClimb = getMaxClimb();
            rcVcopy(params.bmin, polyMesh.bmin);
            rcVcopy(params.bmax, polyMesh.bmax);
            params.cs = config.cs;
            params.ch = config.ch;
            params.buildBvTree = true;
            params.userId = 0;
            params.tileX = 0;
            params.tileY = 0;
            params.tileLayer = 0;
            rcVcopy(params.bmin, config.bmin);
            rcVcopy(params.bmax, config.bmax);

            unsigned char* navData;
            int navDataSize;
            OPENMW_CHECK_DT_RESULT(dtCreateNavMeshData(&params, &navData, &navDataSize));

            NavMeshPtr navMesh(dtAllocNavMesh(), &dtFreeNavMesh);
            OPENMW_CHECK_DT_STATUS(navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA));

            return navMesh;
        }

        osg::Vec3f makeOsgVec3f(const float* values)
        {
            return osg::Vec3f(values[0], values[1], values[2]);
        }

        bool inRange(const osg::Vec3f& v1, const osg::Vec3f& v2, const float r, const float h)
        {
            const auto d = v2 - v1;
            return (d.x() * d.x() + d.z() * d.z()) < r * r && std::abs(d.y()) < h;
        }

        std::vector<dtPolyRef> fixupCorridor(const std::vector<dtPolyRef>& path, const std::vector<dtPolyRef>& visited)
        {
            std::vector<dtPolyRef>::const_iterator furthestVisited;

            // Find furthest common polygon.
            const auto it = std::find_if(path.rbegin(), path.rend(), [&] (dtPolyRef pathValue)
            {
                const auto it = std::find(visited.rbegin(), visited.rend(), pathValue);
                if (it == visited.rend())
                    return false;
                furthestVisited = it.base() - 1;
                return true;
            });

            // If no intersection found just return current path.
            if (it == path.rend())
                return path;
            const auto furthestPath = it.base() - 1;

            // Concatenate paths.

            // visited: A x B
            //            ^ furthestVisited
            //    path: C x D
            //            ^ furthestPath
            //  result: x B D

            std::vector<dtPolyRef> result;
            result.reserve(std::size_t(visited.end() - furthestVisited) + std::size_t(path.end() - furthestPath) - 1);
            std::copy(furthestVisited, visited.end(), std::back_inserter(result));
            std::copy(furthestPath + 1, path.end(), std::back_inserter(result));

            return result;
        }

        // This function checks if the path has a small U-turn, that is,
        // a polygon further in the path is adjacent to the first polygon
        // in the path. If that happens, a shortcut is taken.
        // This can happen if the target (T) location is at tile boundary,
        // and we're (S) approaching it parallel to the tile edge.
        // The choice at the vertex can be arbitrary,
        //  +---+---+
        //  |:::|:::|
        //  +-S-+-T-+
        //  |:::|   | <-- the step can end up in here, resulting U-turn path.
        //  +---+---+
        std::vector<dtPolyRef> fixupShortcuts(const std::vector<dtPolyRef>& path, const dtNavMeshQuery& navQuery)
        {
            if (path.size() < 3)
                return path;

            // Get connected polygons
            const dtMeshTile* tile = 0;
            const dtPoly* poly = 0;
            if (dtStatusFailed(navQuery.getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
                return path;

            const std::size_t maxNeis = 16;
            std::array<dtPolyRef, maxNeis> neis;
            std::size_t nneis = 0;

            for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
            {
                const dtLink* link = &tile->links[k];
                if (link->ref != 0)
                {
                    if (nneis < maxNeis)
                        neis[nneis++] = link->ref;
                }
            }

            // If any of the neighbour polygons is within the next few polygons
            // in the path, short cut to that polygon directly.
            const std::size_t maxLookAhead = 6;
            std::size_t cut = 0;
            for (std::size_t i = std::min(maxLookAhead, path.size()) - 1; i > 1 && cut == 0; i--)
            {
                for (std::size_t j = 0; j < nneis; j++)
                {
                    if (path[i] == neis[j])
                    {
                        cut = i;
                        break;
                    }
                }
            }
            if (cut <= 1)
                return path;

            std::vector<dtPolyRef> result;
            const auto offset = cut - 1;
            result.reserve(1 + path.size() - offset);
            result.push_back(path.front());
            std::copy(path.begin() + std::ptrdiff_t(offset), path.end(), std::back_inserter(result));
            return result;
        }

        struct SteerTarget
        {
            osg::Vec3f steerPos;
            unsigned char steerPosFlag;
            dtPolyRef steerPosRef;
        };

        boost::optional<SteerTarget> getSteerTarget(const dtNavMeshQuery& navQuery, const osg::Vec3f& startPos,
            const osg::Vec3f& endPos, const float minTargetDist, const std::vector<dtPolyRef>& path)
        {
            // Find steer target.
            SteerTarget result;
            const int MAX_STEER_POINTS = 3;
            std::array<float, MAX_STEER_POINTS * 3> steerPath;
            std::array<unsigned char, MAX_STEER_POINTS> steerPathFlags;
            std::array<dtPolyRef, MAX_STEER_POINTS> steerPathPolys;
            int nsteerPath = 0;
            navQuery.findStraightPath(startPos.ptr(), endPos.ptr(), path.data(), int(path.size()), steerPath.data(),
                                      steerPathFlags.data(), steerPathPolys.data(), &nsteerPath, MAX_STEER_POINTS);
            assert(nsteerPath >= 0);
            if (!nsteerPath)
                return boost::none;

            // Find vertex far enough to steer to.
            std::size_t ns = 0;
            while (ns < std::size_t(nsteerPath))
            {
                // Stop at Off-Mesh link or when point is further than slop away.
                if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
                        !inRange(makeOsgVec3f(&steerPath[ns * 3]), startPos, minTargetDist, 1000.0f))
                    break;
                ns++;
            }
            // Failed to find good point to steer to.
            if (ns >= std::size_t(nsteerPath))
                return boost::none;

            dtVcopy(result.steerPos.ptr(), &steerPath[ns * 3]);
            result.steerPos.y() = startPos[1];
            result.steerPosFlag = steerPathFlags[ns];
            result.steerPosRef = steerPathPolys[ns];

            return result;
        }

        std::vector<osg::Vec3f> makeSmoothPath(const dtNavMesh& navMesh, const dtNavMeshQuery& navMeshQuery,
            const dtQueryFilter& filter, const osg::Vec3f& start, const osg::Vec3f& end,
            std::vector<dtPolyRef> polygonPath)
        {
            // Iterate over the path to find smooth path on the detail mesh surface.
            osg::Vec3f iterPos;
            navMeshQuery.closestPointOnPoly(polygonPath.front(), start.ptr(), iterPos.ptr(), 0);

            osg::Vec3f targetPos;
            navMeshQuery.closestPointOnPoly(polygonPath.back(), end.ptr(), targetPos.ptr(), 0);

            const float STEP_SIZE = 0.5f;
            const float SLOP = 0.01f;

            std::vector<osg::Vec3f> smoothPath({iterPos});

            // Move towards target a small advancement at a time until target reached or
            // when ran out of memory to store the path.
            while (!polygonPath.empty())
            {
                // Find location to steer towards.
                const auto steerTarget = getSteerTarget(navMeshQuery, iterPos, targetPos, SLOP, polygonPath);

                if (!steerTarget)
                    break;

                const bool endOfPath = bool(steerTarget->steerPosFlag & DT_STRAIGHTPATH_END);
                const bool offMeshConnection = bool(steerTarget->steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION);

                // Find movement delta.
                const osg::Vec3f delta = steerTarget->steerPos - iterPos;
                float len = delta.length();
                // If the steer target is end of path or off-mesh link, do not move past the location.
                if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
                    len = 1;
                else
                    len = STEP_SIZE / len;

                const osg::Vec3f moveTgt = iterPos + delta * len;

                // Move
                osg::Vec3f result;
                std::vector<dtPolyRef> visited(16);
                int nvisited = 0;
                OPENMW_CHECK_DT_STATUS(navMeshQuery.moveAlongSurface(polygonPath.front(), iterPos.ptr(), moveTgt.ptr(),
                        &filter, result.ptr(), visited.data(), &nvisited, int(visited.size())));

                assert(nvisited >= 0);
                assert(nvisited <= int(visited.size()));
                visited.resize(std::size_t(nvisited));

                polygonPath = fixupCorridor(polygonPath, visited);
                polygonPath = fixupShortcuts(polygonPath, navMeshQuery);

                float h = 0;
                OPENMW_CHECK_DT_STATUS(navMeshQuery.getPolyHeight(polygonPath.front(), result.ptr(), &h));
                result.y() = h;
                iterPos = result;

                // Handle end of path and off-mesh links when close enough.
                if (endOfPath && inRange(iterPos, steerTarget->steerPos, SLOP, 1.0f))
                {
                    // Reached end of path.
                    iterPos = targetPos;
                    smoothPath.push_back(iterPos);
                    break;
                }
                else if (offMeshConnection && inRange(iterPos, steerTarget->steerPos, SLOP, 1.0f))
                {
                    // Advance the path up to and over the off-mesh connection.
                    dtPolyRef prevRef = 0;
                    dtPolyRef polyRef = polygonPath.front();
                    std::size_t npos = 0;
                    while (npos < polygonPath.size() && polyRef != steerTarget->steerPosRef)
                    {
                        prevRef = polyRef;
                        polyRef = polygonPath[npos];
                        ++npos;
                    }
                    std::copy(polygonPath.begin() + std::ptrdiff_t(npos), polygonPath.end(), polygonPath.begin());
                    polygonPath.resize(polygonPath.size() - npos);

                    // Reached off-mesh connection.
                    osg::Vec3f startPos;
                    osg::Vec3f endPos;

                    // Handle the connection.
                    if (dtStatusSucceed(navMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef,
                            startPos.ptr(), endPos.ptr())))
                    {
                        smoothPath.push_back(startPos);

                        // Hack to make the dotted path not visible during off-mesh connection.
                        if (smoothPath.size() & 1)
                        {
                            smoothPath.push_back(startPos);
                        }

                        // Move position at the other side of the off-mesh link.
                        iterPos = endPos;
                        float eh = 0.0f;
                        OPENMW_CHECK_DT_STATUS(navMeshQuery.getPolyHeight(polygonPath.front(), iterPos.ptr(), &eh));
                        iterPos.y() = eh;
                    }
                }

                // Store results.
                smoothPath.push_back(iterPos);
            }

            return smoothPath;
        }

        std::vector<osg::Vec3f> findSmoothPath(const dtNavMesh& navMesh, osg::Vec3f halfExtents,
                                               osg::Vec3f start, osg::Vec3f end,
                                               std::size_t maxPathLength = 1024, int maxNodes = 2048)
        {
            dtNavMeshQuery navMeshQuery;
            OPENMW_CHECK_DT_STATUS(navMeshQuery.init(&navMesh, maxNodes));

            // Detour uses second coordinate as height
            std::swap(start.y(), start.z());
            std::swap(end.y(), end.z());
            std::swap(halfExtents.y(), halfExtents.z());

            dtQueryFilter queryFilter;

            dtPolyRef startRef;
            osg::Vec3f startPolygonPosition;
            OPENMW_CHECK_DT_STATUS(navMeshQuery.findNearestPoly(start.ptr(), halfExtents.ptr(), &queryFilter, &startRef,
                                                                startPolygonPosition.ptr()));

            if (startRef == 0)
                throw NavigatorException("start polygon is not found at " __FILE__ ":" + std::to_string(__LINE__));

            dtPolyRef endRef;
            osg::Vec3f endPolygonPosition;
            OPENMW_CHECK_DT_STATUS(navMeshQuery.findNearestPoly(end.ptr(), halfExtents.ptr(), &queryFilter, &endRef,
                                                                endPolygonPosition.ptr()));

            if (endRef == 0)
                throw NavigatorException("end polygon is not found at " __FILE__ ":" + std::to_string(__LINE__));

            std::vector<dtPolyRef> polygonPath(maxPathLength);
            int pathLen;
            OPENMW_CHECK_DT_STATUS(navMeshQuery.findPath(startRef, endRef, start.ptr(), end.ptr(), &queryFilter,
                                                         polygonPath.data(), &pathLen, int(polygonPath.size())));

            assert(pathLen >= 0);

            polygonPath.resize(std::size_t(pathLen));

            if (polygonPath.empty())
                return std::vector<osg::Vec3f>();

            auto result = makeSmoothPath(navMesh, navMeshQuery, queryFilter, start, end, std::move(polygonPath));

            // Swap coordinates back
            for (auto& v : result)
                std::swap(v.y(), v.z());

            return result;
        }

// Use to dump scene to load from recastnavigation demo tool
#ifdef OPENMW_WRITE_OBJ
        void writeObj(const std::vector<float>& vertices, const std::vector<int>& indices)
        {
            const auto path = std::string("scene.") + std::to_string(std::time(nullptr)) + ".obj";
            std::ofstream file(path);
            if (!file.is_open())
                throw NavigatorException("Open file failed: " + path);
            file.exceptions(std::ios::failbit | std::ios::badbit);
            file.precision(std::numeric_limits<float>::max_digits10);
            std::size_t count = 0;
            for (auto v : vertices)
            {
                if (count % 3 == 0)
                {
                    if (count != 0)
                        file << '\n';
                    file << 'v';
                }
                file << ' ' << v;
                ++count;
            }
            file << '\n';
            count = 0;
            for (auto v : indices)
            {
                if (count % 3 == 0)
                {
                    if (count != 0)
                        file << '\n';
                    file << 'f';
                }
                file << ' ' << (v + 1);
                ++count;
            }
            file << '\n';
        }
#endif
    }

    RecastMeshBuilder& RecastMeshBuilder::addObject(const btConcaveShape& shape, const btTransform& transform)
    {
        auto callback = makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
        {
            for (std::size_t i = 3; i > 0; --i)
                addVertex(transform(triangle[i - 1]) * DetourTraits::recastScaleFactor);
        });
        return addObject(shape, callback);
    }

    RecastMeshBuilder& RecastMeshBuilder::addObject(const btHeightfieldTerrainShape& shape, const btTransform& transform)
    {
        auto callback = makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
        {
            for (std::size_t i = 0; i < 3; ++i)
                addVertex(transform(triangle[i]) * DetourTraits::recastScaleFactor);
        });
        return addObject(shape, callback);
    }

    RecastMesh RecastMeshBuilder::create() const
    {
        return RecastMesh(mIndices, mVertices);
    }

    RecastMeshBuilder& RecastMeshBuilder::addObject(const btConcaveShape& shape, btTriangleCallback& callback)
    {
        btVector3 aabbMin;
        btVector3 aabbMax;
        shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
        shape.processAllTriangles(&callback, aabbMin, aabbMax);
        return *this;
    }

    void RecastMeshBuilder::addVertex(const btVector3& worldPosition)
    {
        mIndices.push_back(int(mIndices.size()));
        mVertices.push_back(worldPosition.x());
        mVertices.push_back(worldPosition.z());
        mVertices.push_back(worldPosition.y());
    }

    bool RecastMeshManager::addObject(std::size_t id, const btHeightfieldTerrainShape& shape, const btTransform& transform)
    {
        if (!mObjects.insert(std::make_pair(id, Object {&shape, transform})).second)
            return false;
        rebuild();
        mMeshBuilder.addObject(shape, transform);
        return true;
    }

    bool RecastMeshManager::addObject(std::size_t id, const btConcaveShape& shape, const btTransform& transform)
    {
        if (const auto concaveShape = dynamic_cast<const btConcaveShape*>(&shape))
        {
            if (!mObjects.insert(std::make_pair(id, Object {&shape, transform})).second)
                return false;
            rebuild();
            mMeshBuilder.addObject(*concaveShape, transform);
            return true;
        }
        return false;
    }

    bool RecastMeshManager::removeObject(std::size_t id)
    {
        if (!mObjects.erase(id))
            return false;
        mShouldRebuild = true;
        return true;
    }

    RecastMesh RecastMeshManager::getMesh()
    {
        rebuild();
        return mMeshBuilder.create();
    }

    void RecastMeshManager::rebuild()
    {
        if (mShouldRebuild)
        {
            mMeshBuilder = RecastMeshBuilder();
            for (const auto& v : mObjects)
                if (const auto heightField = dynamic_cast<const btHeightfieldTerrainShape*>(v.second.mShape))
                    mMeshBuilder.addObject(*heightField, v.second.mTransform);
                else if (const auto concaveShape = dynamic_cast<const btConcaveShape*>(v.second.mShape))
                    mMeshBuilder.addObject(*concaveShape, v.second.mTransform);
            mShouldRebuild = false;
        }
    }

    bool CachedRecastMeshManager::removeObject(std::size_t id)
    {
        if (!mImpl.removeObject(id))
            return false;
        mCached.reset();
        return true;
    }

    const RecastMesh& CachedRecastMeshManager::getMesh()
    {
        if (!mCached.is_initialized())
            mCached = mImpl.getMesh();
#ifdef OPENMW_WRITE_OBJ
        writeObj(mCached->getVertices(), mCached->getIndices());
#endif
        return *mCached;
    }

    AsyncNavMeshMaker::AsyncNavMeshMaker()
        : mThread([&] { process(); })
    {
        mShouldStop = false;
    }

    AsyncNavMeshMaker::~AsyncNavMeshMaker()
    {
        mShouldStop = true;
        std::unique_lock<std::mutex> lock(mMutex);
        mJobs.clear();
        lock.unlock();
        mHasJob.notify_one();
        mThread.join();
    }

    void AsyncNavMeshMaker::post(const osg::Vec3f& agentHalfExtents, RecastMesh recastMesh,
                                   const std::shared_ptr<NavMeshCacheItem>& mNavMeshCacheItem)
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mJobs[agentHalfExtents] = std::make_shared<Job>(Job {agentHalfExtents, std::move(recastMesh), mNavMeshCacheItem});
        lock.unlock();
        mHasJob.notify_one();
    }

    void AsyncNavMeshMaker::process()
    {
        std::cout << "start process jobs" << std::endl;
        while (!mShouldStop)
        {
            std::unique_lock<std::mutex> lock(mMutex);
            if (mJobs.empty())
                mHasJob.wait(lock);
            std::cout << "got " << mJobs.size() << " jobs" << std::endl;
            if (mJobs.empty())
                continue;
            const auto job = mJobs.begin()->second;
            mJobs.erase(mJobs.begin());
            lock.unlock();
            mMaxRevision = std::max(job->mNavMeshCacheItem->mRevision, mMaxRevision);
            std::cout << "process job for agent=" << job->mAgentHalfExtents
                      << " revision=" << job->mNavMeshCacheItem->mRevision
                      << " max_revision=" << mMaxRevision << std::endl;
            if (job->mNavMeshCacheItem->mRevision < mMaxRevision)
                continue;
            try
            {
                job->mNavMeshCacheItem->mValue = makeNavMesh(job->mAgentHalfExtents, job->mRecastMesh);
                std::cout << "cache updated for agent=" << job->mAgentHalfExtents << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cout << e.what() << std::endl;
            }
        }
        std::cout << "stop process jobs" << std::endl;
    }

    bool NavMeshManager::removeObject(std::size_t id)
    {
        if (!mRecastMeshManager.removeObject(id))
            return false;
        ++mRevision;
        return true;
    }

    void NavMeshManager::reset(const osg::Vec3f& agentHalfExtents)
    {
        mCache.erase(agentHalfExtents);
        std::cout << "cache reset for agent=" << agentHalfExtents << std::endl;
    }

    void NavMeshManager::update(const osg::Vec3f& agentHalfExtents)
    {
        auto it = mCache.find(agentHalfExtents);
        if (it == mCache.end())
            it = mCache.insert(std::make_pair(agentHalfExtents, std::make_shared<NavMeshCacheItem>(mRevision))).first;
        else if (it->second->mRevision >= mRevision)
            return;
        it->second->mRevision = mRevision;
        mAsyncNavMeshCreator.post(agentHalfExtents, mRecastMeshManager.getMesh(), it->second);
        std::cout << "cache update posted for agent=" << agentHalfExtents << std::endl;
    }

    NavMeshConstPtr NavMeshManager::getNavMesh(const osg::Vec3f& agentHalfExtents) const
    {
        const auto it = mCache.find(agentHalfExtents);
        if (it == mCache.end())
        {
            std::cout << "cache miss for agent=" << agentHalfExtents << std::endl;
            return nullptr;
        }
        std::cout << "cache hit for agent=" << agentHalfExtents << " value=" << it->second->mValue << std::endl;
        return it->second->mValue;
    }

    void Navigator::addAgent(const osg::Vec3f& agentHalfExtents)
    {
        ++mAgents[agentHalfExtents];
    }

    void Navigator::removeAgent(const osg::Vec3f& agentHalfExtents)
    {
        const auto it = mAgents.find(agentHalfExtents);
        if (it == mAgents.end() || --it->second)
            return;
        mAgents.erase(it);
        mNavMeshManager.reset(agentHalfExtents);
    }

    bool Navigator::removeObject(std::size_t id)
    {
        return mNavMeshManager.removeObject(id);
    }

    void Navigator::update()
    {
        for (const auto& v : mAgents)
            mNavMeshManager.update(v.first);
    }

    std::vector<osg::Vec3f> Navigator::findPath(const osg::Vec3f& agentHalfExtents,
        const osg::Vec3f& start, const osg::Vec3f& end)
    {
        std::cout << "Navigator::findPath agentHalfExtents=" << agentHalfExtents
                  << " start=" << start << " end=" << end << '\n';
        const auto navMesh = mNavMeshManager.getNavMesh(agentHalfExtents);
        if (!navMesh)
            return std::vector<osg::Vec3f>();
        auto result = findSmoothPath(*navMesh, agentHalfExtents,
            start * DetourTraits::recastScaleFactor, end * DetourTraits::recastScaleFactor);
        for (auto& v : result)
            v *= DetourTraits::invertedRecastScaleFactor;
        return result;
    }

#undef OPENMW_CHECK_DT_RESULT
#undef OPENMW_CHECK_DT_STATUS
}
