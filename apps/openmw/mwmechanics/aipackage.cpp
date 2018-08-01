#include "aipackage.hpp"

#include <cmath>

#include <components/esm/loadcell.hpp>
#include <components/esm/loadland.hpp>
#include <components/esm/loadmgef.hpp>

#include <components/detournavigator/navigator.hpp>
#include <components/detournavigator/debug.hpp>

#include "../mwbase/world.hpp"
#include "../mwbase/environment.hpp"

#include "../mwworld/action.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/cellstore.hpp"
#include "../mwworld/inventorystore.hpp"

#include "pathgrid.hpp"
#include "creaturestats.hpp"
#include "movement.hpp"
#include "steering.hpp"
#include "actorutil.hpp"
#include "coordinateconverter.hpp"

#include <osg/Quat>

namespace
{
    osg::Vec3f getDirection(const ESM::Position& position)
    {
        const auto rot = position.rot;
        return osg::Quat(rot[0], -osg::X_AXIS, rot[1], -osg::Y_AXIS, rot[2], -osg::Z_AXIS) * osg::Y_AXIS;
    }

    osg::Vec3f getPosition(const ESM::Position& position)
    {
        return osg::Vec3f(position.pos[0], position.pos[1], position.pos[2]);
    }

    osg::Vec2f getXY(const osg::Vec3f& value)
    {
        return osg::Vec2f(value.x(), value.y());
    }

    float getCos(const osg::Vec2f& lhs, const osg::Vec2f& rhs)
    {
        return (lhs * rhs) / (lhs.length() * rhs.length());
    }
}

MWMechanics::AiPackage::~AiPackage() {}

MWMechanics::AiPackage::AiPackage() :
    mTimer(AI_REACTION_TIME + 1.0f), // to force initial pathbuild
    mTargetActorRefId(""),
    mTargetActorId(-1),
    mIsShortcutting(false),
    mShortcutProhibited(false),
    mShortcutFailPos()
{
}

MWWorld::Ptr MWMechanics::AiPackage::getTarget() const
{
    if (mTargetActorId == -2)
        return MWWorld::Ptr();

    if (mTargetActorId == -1)
    {
        MWWorld::Ptr target = MWBase::Environment::get().getWorld()->searchPtr(mTargetActorRefId, false);
        if (target.isEmpty())
        {
            mTargetActorId = -2;
            return target;
        }
        else
            mTargetActorId = target.getClass().getCreatureStats(target).getActorId();
    }

    if (mTargetActorId != -1)
        return MWBase::Environment::get().getWorld()->searchPtrViaActorId(mTargetActorId);
    else
        return MWWorld::Ptr();
}

bool MWMechanics::AiPackage::sideWithTarget() const
{
    return false;
}

bool MWMechanics::AiPackage::followTargetThroughDoors() const
{
    return false;
}

bool MWMechanics::AiPackage::canCancel() const
{
    return true;
}

bool MWMechanics::AiPackage::shouldCancelPreviousAi() const
{
    return true;
}

bool MWMechanics::AiPackage::getRepeat() const
{
    return false;
}

void MWMechanics::AiPackage::reset()
{
    // reset all members
    mTimer = AI_REACTION_TIME + 1.0f;
    mIsShortcutting = false;
    mShortcutProhibited = false;
    mShortcutFailPos = osg::Vec3f();

    mPathFinder.clearPath();
    mObstacleCheck.clear();
}

bool MWMechanics::AiPackage::pathTo(const MWWorld::Ptr& actor, const osg::Vec3f& destination, const float duration,
        const float destinationTolerance)
{
    using DetourNavigator::Flag;
    using DetourNavigator::Flags;

    mTimer += duration;

    const auto position = actor.getRefData().getPosition().asVec3();

    /// Stops the actor when it gets too close to a unloaded cell
    //... At current time, this test is unnecessary. AI shuts down when actor is more than 7168
    //... units from player, and exterior cells are 8192 units long and wide.
    //... But AI processing distance may increase in the future.
    if (isNearInactiveCell(position))
    {
        actor.getClass().getMovementSettings(actor).mPosition[1] = 0;
        return false;
    }

    const auto pointTolerance = actor.getClass().getSpeed(actor);

    mPathFinder.update(position, pointTolerance, destinationTolerance);

    if (mPathFinder.checkPathCompleted())
    {
        zTurn(actor, getZAngleToPoint(position, destination));
        smoothTurn(actor, getXAngleToPoint(position, destination), 0);
        return true;
    }

    const auto world = MWBase::Environment::get().getWorld();
    const auto navigator = world->getNavigator();
    const Flags navigatorFlags = getNavigatorFlags(actor);

    if (mTimer >= AI_REACTION_TIME)
    {
        if (actor.getClass().isBipedal(actor))
            openDoors(actor);

        if (doesPathNeedRecalc(position, actor.getCell()))
        {
            mPathFinder.buildPath(position, destination, actor.getCell(),
                getPathGridGraph(actor.getCell()));
        }

        mTimer = 0;
    }

    navigator->updateAgentTarget(reinterpret_cast<std::size_t>(actor.getBase()),
        mPathFinder.isPathEmpty() ? destination : mPathFinder.getNextPoint(), navigatorFlags);

    if (mPathFinder.isPathEmpty() || canActorMoveByZ(actor))
    {
        const auto& target = mPathFinder.isPathEmpty() ? destination : mPathFinder.getNextPoint();
        zTurn(actor, getZAngleToPoint(position, target));
        smoothTurn(actor, getXAngleToPoint(position, target), 0);
        actor.getClass().getMovementSettings(actor).mPosition[1] = 1;

        const auto halfExtents = world->getHalfExtents(actor);
        world->updateActorPath(actor, mPathFinder.getPath(), halfExtents, position, target, destination);
    }
    else
    {
        const auto target = navigator->getAgentPosition(reinterpret_cast<std::size_t>(actor.getBase()));
        zTurn(actor, getZAngleToPoint(position, target), 0);
        smoothTurn(actor, getXAngleToPoint(position, target), 0, 0);

        const auto direction = getXY(getDirection(actor.getRefData().getPosition()));
        auto toTarget = getXY(target - getPosition(actor.getRefData().getPosition()));
        toTarget.normalize();
        if (getCos(direction, toTarget) >= 0)
            actor.getClass().getMovementSettings(actor).mPosition[1] = 1;
        else
            actor.getClass().getMovementSettings(actor).mPosition[1] = 0;

        const auto halfExtents = world->getHalfExtents(actor);
        world->updateActorPath(actor, mPathFinder.getPath(), halfExtents, position, target, destination);
    }

    mObstacleCheck.update(actor, duration);

    // handle obstacles on the way
    evadeObstacles(actor);

    return false;
}

void MWMechanics::AiPackage::evadeObstacles(const MWWorld::Ptr& actor)
{
    // check if stuck due to obstacles
    if (!mObstacleCheck.isEvading()) return;

    // first check if obstacle is a door
    static float distance = MWBase::Environment::get().getWorld()->getMaxActivationDistance();

    const MWWorld::Ptr door = getNearbyDoor(actor, distance);
    if (!door.isEmpty() && actor.getClass().isBipedal(actor))
    {
        openDoors(actor);
    }
    else
    {
        mObstacleCheck.takeEvasiveAction(actor.getClass().getMovementSettings(actor));
    }
}

void MWMechanics::AiPackage::openDoors(const MWWorld::Ptr& actor)
{
    static float distance = MWBase::Environment::get().getWorld()->getMaxActivationDistance();

    const MWWorld::Ptr door = getNearbyDoor(actor, distance);
    if (door == MWWorld::Ptr())
        return;

    // note: AiWander currently does not open doors
    if (getTypeId() != TypeIdWander && !door.getCellRef().getTeleport() && door.getClass().getDoorState(door) == 0)
    {
        if ((door.getCellRef().getTrap().empty() && door.getCellRef().getLockLevel() <= 0 ))
        {
            MWBase::Environment::get().getWorld()->activate(door, actor);
            return;
        }

        const std::string keyId = door.getCellRef().getKey();
        if (keyId.empty())
            return;

        MWWorld::ContainerStore &invStore = actor.getClass().getContainerStore(actor);
        MWWorld::Ptr keyPtr = invStore.search(keyId);

        if (!keyPtr.isEmpty())
            MWBase::Environment::get().getWorld()->activate(door, actor);
    }
}

const MWMechanics::PathgridGraph& MWMechanics::AiPackage::getPathGridGraph(const MWWorld::CellStore *cell)
{
    const ESM::CellId& id = cell->getCell()->getCellId();
    // static cache is OK for now, pathgrids can never change during runtime
    typedef std::map<ESM::CellId, std::unique_ptr<MWMechanics::PathgridGraph> > CacheMap;
    static CacheMap cache;
    CacheMap::iterator found = cache.find(id);
    if (found == cache.end())
    {
        cache.insert(std::make_pair(id, std::unique_ptr<MWMechanics::PathgridGraph>(new MWMechanics::PathgridGraph(cell))));
    }
    return *cache[id].get();
}

bool MWMechanics::AiPackage::shortcutPath(const osg::Vec3f& startPoint, const osg::Vec3f& endPoint,
        const MWWorld::Ptr& actor, bool *destInLOS, bool isPathClear)
{
    if (!mShortcutProhibited || (mShortcutFailPos - startPoint).length() >= PATHFIND_SHORTCUT_RETRY_DIST)
    {
        // check if target is clearly visible
        isPathClear = !MWBase::Environment::get().getWorld()->castRay(
            startPoint.x(), startPoint.y(), startPoint.z(),
            endPoint.x(), endPoint.y(), endPoint.z());

        if (destInLOS != NULL) *destInLOS = isPathClear;

        if (!isPathClear)
            return false;

        // check if an actor can move along the shortcut path
        isPathClear = checkWayIsClearForActor(startPoint, endPoint, actor);
    }

    if (isPathClear) // can shortcut the path
    {
        mPathFinder.clearPath();
        mPathFinder.addPointToPath(endPoint);
        return true;
    }

    return false;
}

bool MWMechanics::AiPackage::checkWayIsClearForActor(const osg::Vec3f& startPoint, const osg::Vec3f& endPoint, const MWWorld::Ptr& actor)
{
    const bool actorCanMoveByZ = (actor.getClass().canSwim(actor) && MWBase::Environment::get().getWorld()->isSwimming(actor))
        || MWBase::Environment::get().getWorld()->isFlying(actor);

    if (actorCanMoveByZ)
        return true;

    const float actorSpeed = actor.getClass().getSpeed(actor);
    const float maxAvoidDist = AI_REACTION_TIME * actorSpeed + actorSpeed / MAX_VEL_ANGULAR_RADIANS * 2; // *2 - for reliability
    const float distToTarget = osg::Vec2f(endPoint.x(), endPoint.y()).length();

    const float offsetXY = distToTarget > maxAvoidDist*1.5? maxAvoidDist : maxAvoidDist/2;

    const bool isClear = checkWayIsClear(startPoint, endPoint, offsetXY);

    // update shortcut prohibit state
    if (isClear)
    {
        if (mShortcutProhibited)
        {
            mShortcutProhibited = false;
            mShortcutFailPos = osg::Vec3f();
        }
    }
    if (!isClear)
    {
        if (mShortcutFailPos == osg::Vec3f())
        {
            mShortcutProhibited = true;
            mShortcutFailPos = startPoint;
        }
    }

    return isClear;
}

bool MWMechanics::AiPackage::doesPathNeedRecalc(const osg::Vec3f& newDest, const MWWorld::CellStore* currentCell)
{
    return mPathFinder.getPath().empty()
        || (distance(mPathFinder.getPath().back(), newDest) > 10)
        || mPathFinder.getPathCell() != currentCell;
}

bool MWMechanics::AiPackage::isTargetMagicallyHidden(const MWWorld::Ptr& target)
{
    const MagicEffects& magicEffects(target.getClass().getCreatureStats(target).getMagicEffects());
    return (magicEffects.get(ESM::MagicEffect::Invisibility).getMagnitude() > 0)
        || (magicEffects.get(ESM::MagicEffect::Chameleon).getMagnitude() > 75);
}

bool MWMechanics::AiPackage::isNearInactiveCell(const osg::Vec3f& position) const
{
    const ESM::Cell* playerCell(getPlayer().getCell()->getCell());
    if (playerCell->isExterior())
    {
        // get actor's distance from origin of center cell
        osg::Vec3f actorOffset = position;
        CoordinateConverter(playerCell).toLocal(actorOffset);

        // currently assumes 3 x 3 grid for exterior cells, with player at center cell.
        // ToDo: (Maybe) use "exterior cell load distance" setting to get count of actual active cells
        // While AI Process distance is 7168, AI shuts down actors before they reach edges of 3 x 3 grid.
        const float distanceFromEdge = 200.0;
        float minThreshold = (-1.0f * ESM::Land::REAL_SIZE) + distanceFromEdge;
        float maxThreshold = (2.0f * ESM::Land::REAL_SIZE) - distanceFromEdge;
        return (actorOffset[0] < minThreshold) || (maxThreshold < actorOffset[0])
            || (actorOffset[1] < minThreshold) || (maxThreshold < actorOffset[1]);
    }
    else
    {
        return false;
    }
}

bool MWMechanics::AiPackage::isReachableRotatingOnTheRun(const MWWorld::Ptr& actor, const osg::Vec3f& dest)
{
    // get actor's shortest radius for moving in circle
    float speed = actor.getClass().getSpeed(actor);
    speed += speed * 0.1f; // 10% real speed inaccuracy
    float radius = speed / MAX_VEL_ANGULAR_RADIANS;

    // get radius direction to the center
    const float* rot = actor.getRefData().getPosition().rot;
    osg::Quat quatRot(rot[0], -osg::X_AXIS, rot[1], -osg::Y_AXIS, rot[2], -osg::Z_AXIS);
    osg::Vec3f dir = quatRot * osg::Y_AXIS; // actor's orientation direction is a tangent to circle
    osg::Vec3f radiusDir = dir ^ osg::Z_AXIS; // radius is perpendicular to a tangent
    radiusDir.normalize();
    radiusDir *= radius;

    // pick up the nearest center candidate
    osg::Vec3f pos = actor.getRefData().getPosition().asVec3();
    osg::Vec3f center1 = pos - radiusDir;
    osg::Vec3f center2 = pos + radiusDir;
    osg::Vec3f center = (center1 - dest).length2() < (center2 - dest).length2() ? center1 : center2;

    float distToDest = (center - dest).length();

    // if pathpoint is reachable for the actor rotating on the run:
    // no points of actor's circle should be farther from the center than destination point
    return (radius <= distToDest);
}

DetourNavigator::Flags MWMechanics::AiPackage::getNavigatorFlags(const MWWorld::Ptr& actor) const
{
    const auto& actorClass = actor.getClass();
    DetourNavigator::Flags result = DetourNavigator::Flag_none;

    if (actorClass.isPureWaterCreature(actor) || (getTypeId() != TypeIdWander && actorClass.canSwim(actor)))
        result |= DetourNavigator::Flag_swim;

    if (actorClass.canWalk(actor))
        result |= DetourNavigator::Flag_walk;

    return result;
}

bool MWMechanics::AiPackage::canActorMoveByZ(const MWWorld::Ptr& actor) const
{
    const auto world = MWBase::Environment::get().getWorld();
    const auto& actorClass = actor.getClass();
    return (actorClass.canSwim(actor) && world->isSwimming(actor)) || world->isFlying(actor);
}

bool MWMechanics::AiPackage::isDestinationMoved(const osg::Vec3f& destination) const
{
    return distance(mPathFinder.getLastPoint(), destination) > 10;
}
