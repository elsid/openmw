#ifndef GAME_MWMECHANICS_PATHFINDING_H
#define GAME_MWMECHANICS_PATHFINDING_H

#include <deque>
#include <cassert>

#include <components/esm/defs.hpp>
#include <components/esm/loadpgrd.hpp>

namespace MWWorld
{
    class CellStore;
}

namespace MWMechanics
{
    class PathgridGraph;

    inline float distance(const osg::Vec3f& lhs, const osg::Vec3f& rhs)
    {
        return (lhs - rhs).length();
    }

    inline float getZAngleToDir(const osg::Vec3f& dir)
    {
        return std::atan2(dir.x(), dir.y());
    }

    inline float getXAngleToDir(const osg::Vec3f& dir)
    {
        float dirLen = dir.length();
        return (dirLen != 0) ? -std::asin(dir.z() / dirLen) : 0;
    }

    inline float getZAngleToPoint(const osg::Vec3f& origin, const osg::Vec3f& dest)
    {
        return getZAngleToDir(dest - origin);
    }

    inline float getXAngleToPoint(const osg::Vec3f& origin, const osg::Vec3f& dest)
    {
        return getXAngleToDir(dest - origin);
    }

    const float PATHFIND_Z_REACH = 50.0f;
    //static const float sMaxSlope = 49.0f; // duplicate as in physicssystem
    // distance after which actor (failed previously to shortcut) will try again
    const float PATHFIND_SHORTCUT_RETRY_DIST = 300.0f;

    const float DEFAULT_TOLERANCE = 32.0f;

    // cast up-down ray with some offset from actor position to check for pits/obstacles on the way to target;
    // magnitude of pits/obstacles is defined by PATHFIND_Z_REACH
    bool checkWayIsClear(const osg::Vec3f& from, const osg::Vec3f& to, float offsetXY);

    class PathFinder
    {
        public:
            PathFinder();

            void clearPath();

            void buildPath(const osg::Vec3f& startPoint, const osg::Vec3f& endPoint,
                           const MWWorld::CellStore* cell, const PathgridGraph& pathgridGraph);

            bool checkPathCompleted(const osg::Vec3f& position, const float tolerance = DEFAULT_TOLERANCE);
            ///< \Returns true if we are within \a tolerance units of the last path point.

            /// In radians
            float getZAngleToNext(float x, float y) const;

            float getXAngleToNext(float x, float y, float z) const;

            bool isPathConstructed() const
            {
                return !mPath.empty();
            }

            std::size_t getPathSize() const
            {
                return mPath.size();
            }

            const std::deque<osg::Vec3f>& getPath() const
            {
                return mPath;
            }

            const MWWorld::CellStore* getPathCell() const;

            /** Synchronize new path with old one to avoid visiting 1 waypoint 2 times
            @note
                BuildPath() takes closest PathGrid point to NPC as first point of path.
                This is undesirable if NPC has just passed a Pathgrid point, as this
                makes the 2nd point of the new path == the 1st point of old path.
                Which results in NPC "running in a circle" back to the just passed waypoint.
             */
            void buildSyncedPath(const osg::Vec3f& startPoint, const osg::Vec3f& endPoint,
                const MWWorld::CellStore* cell, const PathgridGraph& pathgridGraph);

            void addPointToPath(const osg::Vec3f& point)
            {
                mPath.push_back(point);
            }

            /// utility function to convert a osg::Vec3f to a Pathgrid::Point
            static ESM::Pathgrid::Point makePathgridPoint(const osg::Vec3f& v)
            {
                return ESM::Pathgrid::Point(static_cast<int>(v[0]), static_cast<int>(v[1]), static_cast<int>(v[2]));
            }

            /// utility function to convert an ESM::Position to a Pathgrid::Point
            static ESM::Pathgrid::Point makePathgridPoint(const ESM::Position& p)
            {
                return ESM::Pathgrid::Point(static_cast<int>(p.pos[0]), static_cast<int>(p.pos[1]), static_cast<int>(p.pos[2]));
            }

            static osg::Vec3f makeOsgVec3(const ESM::Pathgrid::Point& p)
            {
                return osg::Vec3f(static_cast<float>(p.mX), static_cast<float>(p.mY), static_cast<float>(p.mZ));
            }

            // Slightly cheaper version for comparisons.
            // Caller needs to be careful for very short distances (i.e. less than 1)
            // or when accumuating the results i.e. (a + b)^2 != a^2 + b^2
            //
            static float distanceSquared(ESM::Pathgrid::Point point, const osg::Vec3f& pos)
            {
                return (MWMechanics::PathFinder::makeOsgVec3(point) - pos).length2();
            }

            // Return the closest pathgrid point index from the specified position
            // coordinates.  NOTE: Does not check if there is a sensible way to get there
            // (e.g. a cliff in front).
            //
            // NOTE: pos is expected to be in local coordinates, as is grid->mPoints
            //
            static int getClosestPoint(const ESM::Pathgrid* grid, const osg::Vec3f& pos)
            {
                assert(grid && !grid->mPoints.empty());

                float distanceBetween = distanceSquared(grid->mPoints[0], pos);
                int closestIndex = 0;

                // TODO: if this full scan causes performance problems mapping pathgrid
                //       points to a quadtree may help
                for(unsigned int counter = 1; counter < grid->mPoints.size(); counter++)
                {
                    float potentialDistBetween = distanceSquared(grid->mPoints[counter], pos);
                    if(potentialDistBetween < distanceBetween)
                    {
                        distanceBetween = potentialDistBetween;
                        closestIndex = counter;
                    }
                }

                return closestIndex;
            }

        private:
            std::deque<osg::Vec3f> mPath;

            const ESM::Pathgrid *mPathgrid;
            const MWWorld::CellStore* mCell;
    };
}

#endif
