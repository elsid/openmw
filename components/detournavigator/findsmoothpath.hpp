#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_FINDSMOOTHPATH_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_FINDSMOOTHPATH_H

#include "dtstatus.hpp"
#include "exceptions.hpp"
#include "flags.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <boost/optional.hpp>

#include <osg/Vec3f>

#include <vector>

class dtNavMesh;

namespace DetourNavigator
{
    struct Settings;

    inline osg::Vec3f makeOsgVec3f(const float* values)
    {
        return osg::Vec3f(values[0], values[1], values[2]);
    }

    inline bool inRange(const osg::Vec3f& v1, const osg::Vec3f& v2, const float r, const float h)
    {
        const auto d = v2 - v1;
        return (d.x() * d.x() + d.z() * d.z()) < r * r && std::abs(d.y()) < h;
    }

    std::vector<dtPolyRef> fixupCorridor(const std::vector<dtPolyRef>& path, const std::vector<dtPolyRef>& visited);

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
    std::vector<dtPolyRef> fixupShortcuts(const std::vector<dtPolyRef>& path, const dtNavMeshQuery& navQuery);

    struct SteerTarget
    {
        osg::Vec3f steerPos;
        unsigned char steerPosFlag;
        dtPolyRef steerPosRef;
    };

    boost::optional<SteerTarget> getSteerTarget(const dtNavMeshQuery& navQuery, const osg::Vec3f& startPos,
            const osg::Vec3f& endPos, const float minTargetDist, const std::vector<dtPolyRef>& path);

    template <class OutputIterator>
    class OutputTransformIterator
    {
    public:
        OutputTransformIterator(OutputIterator& impl, const Settings& settings)
            : mImpl(impl), mSettings(settings)
        {
        }

        OutputTransformIterator& operator *()
        {
            return *this;
        }

        OutputTransformIterator& operator ++(int)
        {
            mImpl++;
            return *this;
        }

        OutputTransformIterator& operator =(const osg::Vec3f& value)
        {
            *mImpl = fromNavMeshCoordinates(value, mSettings);
            return *this;
        }

    private:
        OutputIterator& mImpl;
        const Settings& mSettings;
    };

    template <class OutputIterator>
    OutputIterator makeSmoothPath(const dtNavMesh& navMesh, const dtNavMeshQuery& navMeshQuery,
            const dtQueryFilter& filter, const osg::Vec3f& start, const osg::Vec3f& end,
            std::vector<dtPolyRef> polygonPath, std::size_t maxSmoothPathSize, OutputIterator out)
    {
        // Iterate over the path to find smooth path on the detail mesh surface.
        osg::Vec3f iterPos;
        navMeshQuery.closestPointOnPoly(polygonPath.front(), start.ptr(), iterPos.ptr(), 0);

        osg::Vec3f targetPos;
        navMeshQuery.closestPointOnPoly(polygonPath.back(), end.ptr(), targetPos.ptr(), 0);

        const float STEP_SIZE = 0.5f;
        const float SLOP = 0.01f;

        *out++ = iterPos;

        std::size_t smoothPathSize = 1;

        // Move towards target a small advancement at a time until target reached or
        // when ran out of memory to store the path.
        while (!polygonPath.empty() && smoothPathSize < maxSmoothPathSize)
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
            visited.resize(static_cast<std::size_t>(nvisited));

            polygonPath = fixupCorridor(polygonPath, visited);
            polygonPath = fixupShortcuts(polygonPath, navMeshQuery);

            float h = 0;
            navMeshQuery.getPolyHeight(polygonPath.front(), result.ptr(), &h);
            result.y() = h;
            iterPos = result;

            // Handle end of path and off-mesh links when close enough.
            if (endOfPath && inRange(iterPos, steerTarget->steerPos, SLOP, 1.0f))
            {
                // Reached end of path.
                iterPos = targetPos;
                *out++ = iterPos;
                ++smoothPathSize;
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
                    *out++ = startPos;
                    ++smoothPathSize;

                    // Hack to make the dotted path not visible during off-mesh connection.
                    if (smoothPathSize & 1)
                    {
                        *out++ = startPos;
                        ++smoothPathSize;
                    }

                    // Move position at the other side of the off-mesh link.
                    iterPos = endPos;
                    float eh = 0.0f;
                    OPENMW_CHECK_DT_STATUS(navMeshQuery.getPolyHeight(polygonPath.front(), iterPos.ptr(), &eh));
                    iterPos.y() = eh;
                }
            }

            // Store results.
            *out++ = iterPos;
            ++smoothPathSize;
        }

        return out;
    }

    template <class OutputIterator>
    OutputIterator findSmoothPath(const dtNavMesh& navMesh, const osg::Vec3f& halfExtents,
            const osg::Vec3f& start, const osg::Vec3f& end, const Settings& settings, OutputIterator out)
    {
        dtNavMeshQuery navMeshQuery;
        OPENMW_CHECK_DT_STATUS(navMeshQuery.init(&navMesh, settings.mMaxNavMeshQueryNodes));

        dtQueryFilter queryFilter;
        queryFilter.setIncludeFlags(Flag_swim | Flag_walk);

        dtPolyRef startRef = 0;
        osg::Vec3f startPolygonPosition;
        for (int i = 0; i < 3; ++i)
        {
            const auto status = navMeshQuery.findNearestPoly(start.ptr(), (halfExtents * (1 << i)).ptr(), &queryFilter,
                &startRef, startPolygonPosition.ptr());
            if (!dtStatusFailed(status) && startRef != 0)
                break;
        }

        if (startRef == 0)
            throw NavigatorException("start polygon is not found at " __FILE__ ":" + std::to_string(__LINE__));

        dtPolyRef endRef = 0;
        osg::Vec3f endPolygonPosition;
        for (int i = 0; i < 3; ++i)
        {
            const auto status = navMeshQuery.findNearestPoly(end.ptr(), (halfExtents * (1 << i)).ptr(), &queryFilter,
                &endRef, endPolygonPosition.ptr());
            if (!dtStatusFailed(status) && endRef != 0)
                break;
        }

        if (endRef == 0)
            throw NavigatorException("end polygon is not found at " __FILE__ ":" + std::to_string(__LINE__));

        std::vector<dtPolyRef> polygonPath(settings.mMaxPolygonPathSize);
        int pathLen = 0;
        OPENMW_CHECK_DT_STATUS(navMeshQuery.findPath(startRef, endRef, start.ptr(), end.ptr(), &queryFilter,
            polygonPath.data(), &pathLen, static_cast<int>(polygonPath.size())));

        assert(pathLen >= 0);

        polygonPath.resize(static_cast<std::size_t>(pathLen));

        if (polygonPath.empty())
            return out;

        makeSmoothPath(navMesh, navMeshQuery, queryFilter, start, end, std::move(polygonPath),
            settings.mMaxSmoothPathSize, OutputTransformIterator<OutputIterator>(out, settings));

        return out;
    }
}

#endif
