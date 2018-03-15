#include "physicssystem.hpp"

#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <fstream>

#include <boost/optional.hpp>

#include <osg/Group>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>

#include <LinearMath/btQuickprof.h>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <Recast.h>

#include <components/nifbullet/bulletnifloader.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/bulletshapemanager.hpp>

#include <components/esm/loadgmst.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/unrefqueue.hpp>

#include <components/nifosg/particle.hpp> // FindRecIndexVisitor

#include "../mwbase/world.hpp"
#include "../mwbase/environment.hpp"

#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/movement.hpp"
#include "../mwmechanics/actorutil.hpp"

#include "../mwworld/esmstore.hpp"
#include "../mwworld/cellstore.hpp"

#include "../mwrender/bulletdebugdraw.hpp"

#include "../mwbase/world.hpp"
#include "../mwbase/environment.hpp"

#include "../mwworld/class.hpp"

#include "collisiontype.hpp"
#include "actor.hpp"
#include "convert.hpp"
#include "trace.h"

namespace MWPhysics
{

    static const float sMaxSlope = 49.0f;
    static const float sStepSizeUp = 34.0f;
    static const float sStepSizeDown = 62.0f;
    static const float sMinStep = 10.f;
    static const float sGroundOffset = 1.0f;

    // Arbitrary number. To prevent infinite loops. They shouldn't happen but it's good to be prepared.
    static const int sMaxIterations = 8;

    static bool isActor(const btCollisionObject *obj)
    {
        assert(obj);
        return obj->getBroadphaseHandle()->m_collisionFilterGroup == CollisionType_Actor;
    }

    template <class Vec3>
    static bool isWalkableSlope(const Vec3 &normal)
    {
        static const float sMaxSlopeCos = std::cos(osg::DegreesToRadians(sMaxSlope));
        return (normal.z() > sMaxSlopeCos);
    }

    static bool canStepDown(const ActorTracer &stepper)
    {
        return stepper.mHitObject && isWalkableSlope(stepper.mPlaneNormal) && !isActor(stepper.mHitObject);
    }

    class Stepper
    {
    private:
        const btCollisionWorld *mColWorld;
        const btCollisionObject *mColObj;

        ActorTracer mTracer, mUpStepper, mDownStepper;
        bool mHaveMoved;

    public:
        Stepper(const btCollisionWorld *colWorld, const btCollisionObject *colObj)
            : mColWorld(colWorld)
            , mColObj(colObj)
            , mHaveMoved(true)
        {}

        bool step(osg::Vec3f &position, const osg::Vec3f &toMove, float &remainingTime)
        {
            /*
             * Slide up an incline or set of stairs.  Should be called only after a
             * collision detection otherwise unnecessary tracing will be performed.
             *
             * NOTE: with a small change this method can be used to step over an obstacle
             * of height sStepSize.
             *
             * If successful return 'true' and update 'position' to the new possible
             * location and adjust 'remainingTime'.
             *
             * If not successful return 'false'.  May fail for these reasons:
             *    - can't move directly up from current position
             *    - having moved up by between epsilon() and sStepSize, can't move forward
             *    - having moved forward by between epsilon() and toMove,
             *        = moved down between 0 and just under sStepSize but slope was too steep, or
             *        = moved the full sStepSize down (FIXME: this could be a bug)
             *
             *
             *
             * Starting position.  Obstacle or stairs with height upto sStepSize in front.
             *
             *     +--+                          +--+       |XX
             *     |  | -------> toMove          |  |    +--+XX
             *     |  |                          |  |    |XXXXX
             *     |  | +--+                     |  | +--+XXXXX
             *     |  | |XX|                     |  | |XXXXXXXX
             *     +--+ +--+                     +--+ +--------
             *    ==============================================
             */

            /*
             * Try moving up sStepSize using stepper.
             * FIXME: does not work in case there is no front obstacle but there is one above
             *
             *     +--+                         +--+
             *     |  |                         |  |
             *     |  |                         |  |       |XX
             *     |  |                         |  |    +--+XX
             *     |  |                         |  |    |XXXXX
             *     +--+ +--+                    +--+ +--+XXXXX
             *          |XX|                         |XXXXXXXX
             *          +--+                         +--------
             *    ==============================================
             */
            if (mHaveMoved)
            {
                mHaveMoved = false;
                mUpStepper.doTrace(mColObj, position, position+osg::Vec3f(0.0f,0.0f,sStepSizeUp), mColWorld);
                if(mUpStepper.mFraction < std::numeric_limits<float>::epsilon())
                    return false; // didn't even move the smallest representable amount
                                  // (TODO: shouldn't this be larger? Why bother with such a small amount?)
            }

            /*
             * Try moving from the elevated position using tracer.
             *
             *                          +--+  +--+
             *                          |  |  |YY|   FIXME: collision with object YY
             *                          |  |  +--+
             *                          |  |
             *     <------------------->|  |
             *          +--+            +--+
             *          |XX|      the moved amount is toMove*tracer.mFraction
             *          +--+
             *    ==============================================
             */
            osg::Vec3f tracerPos = mUpStepper.mEndPos;
            mTracer.doTrace(mColObj, tracerPos, tracerPos + toMove, mColWorld);
            if(mTracer.mFraction < std::numeric_limits<float>::epsilon())
                return false; // didn't even move the smallest representable amount

            /*
             * Try moving back down sStepSizeDown using stepper.
             * NOTE: if there is an obstacle below (e.g. stairs), we'll be "stepping up".
             * Below diagram is the case where we "stepped over" an obstacle in front.
             *
             *                                +--+
             *                                |YY|
             *                          +--+  +--+
             *                          |  |
             *                          |  |
             *          +--+            |  |
             *          |XX|            |  |
             *          +--+            +--+
             *    ==============================================
             */
            mDownStepper.doTrace(mColObj, mTracer.mEndPos, mTracer.mEndPos-osg::Vec3f(0.0f,0.0f,sStepSizeDown), mColWorld);
            if (!canStepDown(mDownStepper))
            {
                // Try again with increased step length
                if (mTracer.mFraction < 1.0f || toMove.length2() > sMinStep*sMinStep)
                    return false;

                osg::Vec3f direction = toMove;
                direction.normalize();
                mTracer.doTrace(mColObj, tracerPos, tracerPos + direction*sMinStep, mColWorld);
                if (mTracer.mFraction < 0.001f)
                    return false;

                mDownStepper.doTrace(mColObj, mTracer.mEndPos, mTracer.mEndPos-osg::Vec3f(0.0f,0.0f,sStepSizeDown), mColWorld);
                if (!canStepDown(mDownStepper))
                    return false;
            }
            if (mDownStepper.mFraction < 1.0f)
            {
                // only step down onto semi-horizontal surfaces. don't step down onto the side of a house or a wall.
                // TODO: stepper.mPlaneNormal does not appear to be reliable - needs more testing
                // NOTE: caller's variables 'position' & 'remainingTime' are modified here
                position = mDownStepper.mEndPos;
                remainingTime *= (1.0f-mTracer.mFraction); // remaining time is proportional to remaining distance
                mHaveMoved = true;
                return true;
            }
            return false;
        }
    };

    class MovementSolver
    {
    private:
        ///Project a vector u on another vector v
        static inline osg::Vec3f project(const osg::Vec3f& u, const osg::Vec3f &v)
        {
            return v * (u * v);
            //            ^ dot product
        }

        ///Helper for computing the character sliding
        static inline osg::Vec3f slide(const osg::Vec3f& direction, const osg::Vec3f &planeNormal)
        {
            return direction - project(direction, planeNormal);
        }

    public:
        static osg::Vec3f traceDown(const MWWorld::Ptr &ptr, const osg::Vec3f& position, Actor* actor, btCollisionWorld* collisionWorld, float maxHeight)
        {
            osg::Vec3f offset = actor->getCollisionObjectPosition() - ptr.getRefData().getPosition().asVec3();

            ActorTracer tracer;
            tracer.findGround(actor, position + offset, position + offset - osg::Vec3f(0,0,maxHeight), collisionWorld);
            if(tracer.mFraction >= 1.0f)
            {
                actor->setOnGround(false);
                return position;
            }
            else
            {
                actor->setOnGround(true);

                // Check if we actually found a valid spawn point (use an infinitely thin ray this time).
                // Required for some broken door destinations in Morrowind.esm, where the spawn point
                // intersects with other geometry if the actor's base is taken into account
                btVector3 from = toBullet(position);
                btVector3 to = from - btVector3(0,0,maxHeight);

                btCollisionWorld::ClosestRayResultCallback resultCallback1(from, to);
                resultCallback1.m_collisionFilterGroup = 0xff;
                resultCallback1.m_collisionFilterMask = CollisionType_World|CollisionType_HeightMap;

                collisionWorld->rayTest(from, to, resultCallback1);

                if (resultCallback1.hasHit() &&
                        ( (toOsg(resultCallback1.m_hitPointWorld) - (tracer.mEndPos-offset)).length2() > 35*35
                        || !isWalkableSlope(tracer.mPlaneNormal)))
                {
                    actor->setOnSlope(!isWalkableSlope(resultCallback1.m_hitNormalWorld));
                    return toOsg(resultCallback1.m_hitPointWorld) + osg::Vec3f(0.f, 0.f, sGroundOffset);
                }
                else
                {
                    actor->setOnSlope(!isWalkableSlope(tracer.mPlaneNormal));
                }

                return tracer.mEndPos-offset + osg::Vec3f(0.f, 0.f, sGroundOffset);
            }
        }

        static osg::Vec3f move(osg::Vec3f position, const MWWorld::Ptr &ptr, Actor* physicActor, const osg::Vec3f &movement, float time,
                                  bool isFlying, float waterlevel, float slowFall, const btCollisionWorld* collisionWorld,
                               std::map<MWWorld::Ptr, MWWorld::Ptr>& standingCollisionTracker)
        {
            const ESM::Position& refpos = ptr.getRefData().getPosition();
            // Early-out for totally static creatures
            // (Not sure if gravity should still apply?)
            if (!ptr.getClass().isMobile(ptr))
                return position;

            // Reset per-frame data
            physicActor->setWalkingOnWater(false);
            // Anything to collide with?
            if(!physicActor->getCollisionMode())
            {
                return position +  (osg::Quat(refpos.rot[0], osg::Vec3f(-1, 0, 0)) *
                                    osg::Quat(refpos.rot[2], osg::Vec3f(0, 0, -1))
                                    ) * movement * time;
            }

            const btCollisionObject *colobj = physicActor->getCollisionObject();
            osg::Vec3f halfExtents = physicActor->getHalfExtents();

            // NOTE: here we don't account for the collision box translation (i.e. physicActor->getPosition() - refpos.pos).
            // That means the collision shape used for moving this actor is in a different spot than the collision shape
            // other actors are using to collide against this actor.
            // While this is strictly speaking wrong, it's needed for MW compatibility.
            position.z() += halfExtents.z();

            static const float fSwimHeightScale = MWBase::Environment::get().getWorld()->getStore().get<ESM::GameSetting>()
                    .find("fSwimHeightScale")->getFloat();
            float swimlevel = waterlevel + halfExtents.z() - (physicActor->getRenderingHalfExtents().z() * 2 * fSwimHeightScale);

            ActorTracer tracer;
            osg::Vec3f inertia = physicActor->getInertialForce();
            osg::Vec3f velocity;

            if(position.z() < swimlevel || isFlying)
            {
                velocity = (osg::Quat(refpos.rot[0], osg::Vec3f(-1, 0, 0)) *
                            osg::Quat(refpos.rot[2], osg::Vec3f(0, 0, -1))) * movement;
            }
            else
            {
                velocity = (osg::Quat(refpos.rot[2], osg::Vec3f(0, 0, -1))) * movement;

                if (velocity.z() > 0.f && physicActor->getOnGround() && !physicActor->getOnSlope())
                    inertia = velocity;
                else if(!physicActor->getOnGround() || physicActor->getOnSlope())
                    velocity = velocity + physicActor->getInertialForce();
            }

            // dead actors underwater will float to the surface, if the CharacterController tells us to do so
            if (movement.z() > 0 && ptr.getClass().getCreatureStats(ptr).isDead() && position.z() < swimlevel)
                velocity = osg::Vec3f(0,0,1) * 25;

            ptr.getClass().getMovementSettings(ptr).mPosition[2] = 0;

            // Now that we have the effective movement vector, apply wind forces to it
            if (MWBase::Environment::get().getWorld()->isInStorm())
            {
                osg::Vec3f stormDirection = MWBase::Environment::get().getWorld()->getStormDirection();
                float angleDegrees = osg::RadiansToDegrees(std::acos(stormDirection * velocity / (stormDirection.length() * velocity.length())));
                static const float fStromWalkMult = MWBase::Environment::get().getWorld()->getStore().get<ESM::GameSetting>()
                        .find("fStromWalkMult")->getFloat();
                velocity *= 1.f-(fStromWalkMult * (angleDegrees/180.f));
            }

            Stepper stepper(collisionWorld, colobj);
            osg::Vec3f origVelocity = velocity;
            osg::Vec3f newPosition = position;
            /*
             * A loop to find newPosition using tracer, if successful different from the starting position.
             * nextpos is the local variable used to find potential newPosition, using velocity and remainingTime
             * The initial velocity was set earlier (see above).
             */
            float remainingTime = time;
            for(int iterations = 0; iterations < sMaxIterations && remainingTime > 0.01f; ++iterations)
            {
                osg::Vec3f nextpos = newPosition + velocity * remainingTime;

                // If not able to fly, don't allow to swim up into the air
                if(!isFlying &&                   // can't fly
                   nextpos.z() > swimlevel &&     // but about to go above water
                   newPosition.z() < swimlevel)
                {
                    const osg::Vec3f down(0,0,-1);
                    velocity = slide(velocity, down);
                    // NOTE: remainingTime is unchanged before the loop continues
                    continue; // velocity updated, calculate nextpos again
                }

                if((newPosition - nextpos).length2() > 0.0001)
                {
                    // trace to where character would go if there were no obstructions
                    tracer.doTrace(colobj, newPosition, nextpos, collisionWorld);

                    // check for obstructions
                    if(tracer.mFraction >= 1.0f)
                    {
                        newPosition = tracer.mEndPos; // ok to move, so set newPosition
                        break;
                    }
                }
                else
                {
                    // The current position and next position are nearly the same, so just exit.
                    // Note: Bullet can trigger an assert in debug modes if the positions
                    // are the same, since that causes it to attempt to normalize a zero
                    // length vector (which can also happen with nearly identical vectors, since
                    // precision can be lost due to any math Bullet does internally). Since we
                    // aren't performing any collision detection, we want to reject the next
                    // position, so that we don't slowly move inside another object.
                    break;
                }

                // We are touching something.
                if (tracer.mFraction < 1E-9f)
                {
                    // Try to separate by backing off slighly to unstuck the solver
                    osg::Vec3f backOff = (newPosition - tracer.mHitPoint) * 1E-2f;
                    newPosition += backOff;
                }

                // We hit something. Check if we can step up.
                float hitHeight = tracer.mHitPoint.z() - tracer.mEndPos.z() + halfExtents.z();
                osg::Vec3f oldPosition = newPosition;
                bool result = false;
                if (hitHeight < sStepSizeUp && !isActor(tracer.mHitObject))
                {
                    // Try to step up onto it.
                    // NOTE: stepMove does not allow stepping over, modifies newPosition if successful
                    result = stepper.step(newPosition, velocity*remainingTime, remainingTime);
                }
                if (result)
                {
                    // don't let pure water creatures move out of water after stepMove
                    if (ptr.getClass().isPureWaterCreature(ptr)
                            && newPosition.z() + halfExtents.z() > waterlevel)
                        newPosition = oldPosition;
                }
                else
                {
                    // Can't move this way, try to find another spot along the plane
                    osg::Vec3f newVelocity = slide(velocity, tracer.mPlaneNormal);

                    // Do not allow sliding upward if there is gravity.
                    // Stepping will have taken care of that.
                    if(!(newPosition.z() < swimlevel || isFlying))
                        newVelocity.z() = std::min(newVelocity.z(), 0.0f);

                    if ((newVelocity-velocity).length2() < 0.01)
                        break;
                    if ((newVelocity * origVelocity) <= 0.f)
                        break; // ^ dot product

                    velocity = newVelocity;
                }
            }

            bool isOnGround = false;
            bool isOnSlope = false;
            if (!(inertia.z() > 0.f) && !(newPosition.z() < swimlevel))
            {
                osg::Vec3f from = newPosition;
                osg::Vec3f to = newPosition - (physicActor->getOnGround() ?
                             osg::Vec3f(0,0,sStepSizeDown + 2*sGroundOffset) : osg::Vec3f(0,0,2*sGroundOffset));
                tracer.doTrace(colobj, from, to, collisionWorld);
                if(tracer.mFraction < 1.0f
                        && tracer.mHitObject->getBroadphaseHandle()->m_collisionFilterGroup != CollisionType_Actor)
                {
                    const btCollisionObject* standingOn = tracer.mHitObject;
                    PtrHolder* ptrHolder = static_cast<PtrHolder*>(standingOn->getUserPointer());
                    if (ptrHolder)
                        standingCollisionTracker[ptr] = ptrHolder->getPtr();

                    if (standingOn->getBroadphaseHandle()->m_collisionFilterGroup == CollisionType_Water)
                        physicActor->setWalkingOnWater(true);
                    if (!isFlying)
                        newPosition.z() = tracer.mEndPos.z() + sGroundOffset;

                    isOnGround = true;

                    isOnSlope = !isWalkableSlope(tracer.mPlaneNormal);
                }
                else
                {
                    // standing on actors is not allowed (see above).
                    // in addition to that, apply a sliding effect away from the center of the actor,
                    // so that we do not stay suspended in air indefinitely.
                    if (tracer.mFraction < 1.0f && tracer.mHitObject->getBroadphaseHandle()->m_collisionFilterGroup == CollisionType_Actor)
                    {
                        if (osg::Vec3f(velocity.x(), velocity.y(), 0).length2() < 100.f*100.f)
                        {
                            btVector3 aabbMin, aabbMax;
                            tracer.mHitObject->getCollisionShape()->getAabb(tracer.mHitObject->getWorldTransform(), aabbMin, aabbMax);
                            btVector3 center = (aabbMin + aabbMax) / 2.f;
                            inertia = osg::Vec3f(position.x() - center.x(), position.y() - center.y(), 0);
                            inertia.normalize();
                            inertia *= 100;
                        }
                    }

                    isOnGround = false;
                }
            }

            if((isOnGround && !isOnSlope) || newPosition.z() < swimlevel || isFlying)
                physicActor->setInertialForce(osg::Vec3f(0.f, 0.f, 0.f));
            else
            {
                inertia.z() += time * -627.2f;
                if (inertia.z() < 0)
                    inertia.z() *= slowFall;
                if (slowFall < 1.f) {
                    inertia.x() *= slowFall;
                    inertia.y() *= slowFall;
                }
                physicActor->setInertialForce(inertia);
            }
            physicActor->setOnGround(isOnGround);
            physicActor->setOnSlope(isOnSlope);

            newPosition.z() -= halfExtents.z(); // remove what was added at the beginning
            return newPosition;
        }
    };


    // ---------------------------------------------------------------

    class HeightField
    {
    public:
        HeightField(const float* heights, int x, int y, float triSize, float sqrtVerts, float minH, float maxH, const osg::Object* holdObject)
        {
            mShape = new btHeightfieldTerrainShape(
                sqrtVerts, sqrtVerts, heights, 1,
                minH, maxH, 2,
                PHY_FLOAT, false
            );
            mShape->setUseDiamondSubdivision(true);
            mShape->setLocalScaling(btVector3(triSize, triSize, 1));

            btTransform transform(btQuaternion::getIdentity(),
                                  btVector3((x+0.5f) * triSize * (sqrtVerts-1),
                                            (y+0.5f) * triSize * (sqrtVerts-1),
                                            (maxH+minH)*0.5f));

            mCollisionObject = new btCollisionObject;
            mCollisionObject->setCollisionShape(mShape);
            mCollisionObject->setWorldTransform(transform);

            mHoldObject = holdObject;
        }
        ~HeightField()
        {
            delete mCollisionObject;
            delete mShape;
        }
        btCollisionObject* getCollisionObject()
        {
            return mCollisionObject;
        }
        const btHeightfieldTerrainShape* getShape() const
        {
            return mShape;
        }

    private:
        btHeightfieldTerrainShape* mShape;
        btCollisionObject* mCollisionObject;
        osg::ref_ptr<const osg::Object> mHoldObject;

        void operator=(const HeightField&);
        HeightField(const HeightField&);
    };

    // --------------------------------------------------------------

    class Object : public PtrHolder
    {
    public:
        Object(const MWWorld::Ptr& ptr, osg::ref_ptr<Resource::BulletShapeInstance> shapeInstance)
            : mShapeInstance(shapeInstance)
            , mSolid(true)
        {
            mPtr = ptr;

            mCollisionObject.reset(new btCollisionObject);
            mCollisionObject->setCollisionShape(shapeInstance->getCollisionShape());

            mCollisionObject->setUserPointer(static_cast<PtrHolder*>(this));

            setScale(ptr.getCellRef().getScale());
            setRotation(toBullet(ptr.getRefData().getBaseNode()->getAttitude()));
            const float* pos = ptr.getRefData().getPosition().pos;
            setOrigin(btVector3(pos[0], pos[1], pos[2]));
        }

        const Resource::BulletShapeInstance* getShapeInstance() const
        {
            return mShapeInstance.get();
        }

        void setScale(float scale)
        {
            mShapeInstance->getCollisionShape()->setLocalScaling(btVector3(scale,scale,scale));
        }

        void setRotation(const btQuaternion& quat)
        {
            mCollisionObject->getWorldTransform().setRotation(quat);
        }

        void setOrigin(const btVector3& vec)
        {
            mCollisionObject->getWorldTransform().setOrigin(vec);
        }

        btCollisionObject* getCollisionObject()
        {
            return mCollisionObject.get();
        }

        const btCollisionObject* getCollisionObject() const
        {
            return mCollisionObject.get();
        }

        /// Return solid flag. Not used by the object itself, true by default.
        bool isSolid() const
        {
            return mSolid;
        }

        void setSolid(bool solid)
        {
            mSolid = solid;
        }

        bool isAnimated() const
        {
            return !mShapeInstance->mAnimatedShapes.empty();
        }

        void animateCollisionShapes(btCollisionWorld* collisionWorld)
        {
            if (mShapeInstance->mAnimatedShapes.empty())
                return;

            assert (mShapeInstance->getCollisionShape()->isCompound());

            btCompoundShape* compound = static_cast<btCompoundShape*>(mShapeInstance->getCollisionShape());

            for (std::map<int, int>::const_iterator it = mShapeInstance->mAnimatedShapes.begin(); it != mShapeInstance->mAnimatedShapes.end(); ++it)
            {
                int recIndex = it->first;
                int shapeIndex = it->second;

                std::map<int, osg::NodePath>::iterator nodePathFound = mRecIndexToNodePath.find(recIndex);
                if (nodePathFound == mRecIndexToNodePath.end())
                {
                    NifOsg::FindGroupByRecIndex visitor(recIndex);
                    mPtr.getRefData().getBaseNode()->accept(visitor);
                    if (!visitor.mFound)
                    {
                        std::cerr << "Error: animateCollisionShapes can't find node " << recIndex << " for " << mPtr.getCellRef().getRefId() << std::endl;
                        return;
                    }
                    osg::NodePath nodePath = visitor.mFoundPath;
                    nodePath.erase(nodePath.begin());
                    nodePathFound = mRecIndexToNodePath.insert(std::make_pair(recIndex, nodePath)).first;
                }

                osg::NodePath& nodePath = nodePathFound->second;
                osg::Matrixf matrix = osg::computeLocalToWorld(nodePath);
                osg::Vec3f scale = matrix.getScale();
                matrix.orthoNormalize(matrix);

                btTransform transform;
                transform.setOrigin(toBullet(matrix.getTrans()) * compound->getLocalScaling());
                for (int i=0; i<3; ++i)
                    for (int j=0; j<3; ++j)
                        transform.getBasis()[i][j] = matrix(j,i); // NB column/row major difference

                if (compound->getLocalScaling() * toBullet(scale) != compound->getChildShape(shapeIndex)->getLocalScaling())
                    compound->getChildShape(shapeIndex)->setLocalScaling(compound->getLocalScaling() * toBullet(scale));
                if (!(transform == compound->getChildTransform(shapeIndex)))
                    compound->updateChildTransform(shapeIndex, transform);
            }

            collisionWorld->updateSingleAabb(mCollisionObject.get());
        }

    private:
        std::unique_ptr<btCollisionObject> mCollisionObject;
        osg::ref_ptr<Resource::BulletShapeInstance> mShapeInstance;
        std::map<int, osg::NodePath> mRecIndexToNodePath;
        bool mSolid;
    };

    // ---------------------------------------------------------------

    // Scale all coordinates to change order of values to make rcCreateHeightfield work
    static const float invertedRecastScaleFactor = 64.0f;
    static const float recastScaleFactor = 1.0f / invertedRecastScaleFactor;

    class RecastMesh
    {
    public:
        RecastMesh(std::vector<int> indices, std::vector<float> vertices, std::size_t trianglesCount)
            : mIndices(std::move(indices))
            , mVertices(std::move(vertices))
            , mTrianglesCount(trianglesCount)
        {}

        const std::vector<int>& getIndices() const
        {
            return mIndices;
        }

        const std::vector<float>& getVertices() const
        {
            return mVertices;
        }

        std::size_t getVerticesCount() const
        {
            return std::size_t(mVertices.size() / 3);
        }

        std::size_t getTrianglesCount() const
        {
            return mTrianglesCount;
        }

    private:
        std::vector<int> mIndices;
        std::vector<float> mVertices;
        std::size_t mTrianglesCount;
    };

    template <class Impl>
    class ProcessTriangleCallback : public btTriangleCallback
    {
    public:
        ProcessTriangleCallback(Impl impl)
            : mImpl(std::move(impl))
        {}

        void processTriangle(btVector3* triangle, int partId, int triangleIndex) override final
        {
            return mImpl(triangle, partId, triangleIndex);
        }

    private:
        Impl mImpl;
    };

    template <class Impl>
    ProcessTriangleCallback<typename std::decay<Impl>::type> makeProcessTriangleCallback(Impl&& impl)
    {
        return ProcessTriangleCallback<typename std::decay<Impl>::type>(std::forward<Impl>(impl));
    }

    class RecastMeshBuilder
    {
    public:
        RecastMeshBuilder& addShape(const btConcaveShape& shape, const btTransform& transform)
        {
            auto callback = makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
            {
                for (std::size_t i = 3; i > 0; --i)
                    addVertex(transform(triangle[i - 1]) * recastScaleFactor);
                ++mTrianglesCount;
            });
            return addShape(shape, callback);
        }

        RecastMeshBuilder& addShape(const btHeightfieldTerrainShape& shape, const btTransform& transform)
        {
            auto callback = makeProcessTriangleCallback([&] (btVector3* triangle, int, int)
            {
                for (std::size_t i = 0; i < 3; ++i)
                    addVertex(transform(triangle[i]) * recastScaleFactor);
                ++mTrianglesCount;
            });
            return addShape(shape, callback);
        }

        RecastMesh create()
        {
            return RecastMesh(mIndices, mVertices, mTrianglesCount);
        }

    private:
        std::vector<int> mIndices;
        std::vector<float> mVertices;
        std::size_t mTrianglesCount = 0;
        int mIndex = 0;

        RecastMeshBuilder& addShape(const btConcaveShape& shape, btTriangleCallback& callback)
        {
            btVector3 aabbMin;
            btVector3 aabbMax;
            shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
            shape.processAllTriangles(&callback, aabbMin, aabbMax);
            return *this;
        }

        void addVertex(const btVector3& worldPosition)
        {
            mIndices.push_back(mIndex++);
            mVertices.push_back(worldPosition.x());
            mVertices.push_back(worldPosition.z());
            mVertices.push_back(worldPosition.y());
        }
    };

    class RecastMeshManager
    {
    public:
        bool addObject(HeightField* object)
        {
            if (!mHeightFields.insert(object).second)
                return false;
            mMeshBuilder.addShape(*object->getShape(), object->getCollisionObject()->getWorldTransform());
            return true;
        }

        bool removeObject(HeightField* object)
        {
            if (!mHeightFields.erase(object))
                return false;
            rebuild();
            return true;
        }

        bool addObject(Object* object)
        {
            if (const auto concaveShape = dynamic_cast<btConcaveShape*>(object->getShapeInstance()->mCollisionShape))
            {
                if (!mObjects.insert(object).second)
                    return false;
                mMeshBuilder.addShape(*concaveShape, object->getCollisionObject()->getWorldTransform());
                return true;
            }
            return false;
        }

        bool removeObject(Object* object)
        {
            if (const auto concaveShape = dynamic_cast<btConcaveShape*>(object->getShapeInstance()->mCollisionShape))
            {
                if (!mObjects.erase(object))
                    return false;
                rebuild();
                return true;
            }
            return false;
        }

        RecastMesh getMesh()
        {
            return mMeshBuilder.create();
        }

    private:
        RecastMeshBuilder mMeshBuilder;
        std::unordered_set<HeightField*> mHeightFields;
        std::unordered_set<Object*> mObjects;

        void rebuild()
        {
            mMeshBuilder = RecastMeshBuilder();
            for (auto v : mHeightFields)
                mMeshBuilder.addShape(*v->getShape(), v->getCollisionObject()->getWorldTransform());
            for (auto v : mObjects)
                if (const auto concaveShape = dynamic_cast<btConcaveShape*>(v->getShapeInstance()->mCollisionShape))
                    mMeshBuilder.addShape(*concaveShape, v->getCollisionObject()->getWorldTransform());
        }
    };

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

    class RecastMeshManagerCache
    {
    public:
        template <class T>
        bool addObject(T* object)
        {
            if (!mImpl.addObject(object))
                return false;
            mCached.reset();
            return true;
        }

        template <class T>
        bool removeObject(T* object)
        {
            if (!mImpl.removeObject(object))
                return false;
            mCached.reset();
            return true;
        }

        const RecastMesh& getMesh()
        {
            if (!mCached.is_initialized())
                mCached = mImpl.getMesh();
#ifdef OPENMW_WRITE_OBJ
            writeObj(mCached->getVertices(), mCached->getIndices());
#endif
            return *mCached;
        }

    private:
        RecastMeshManager mImpl;
        boost::optional<RecastMesh> mCached;
    };

    struct AgentParams
    {
        float mHeight;
        float mMaxClimb;
        float mRadius;
    };

    std::tuple<float, float, float> makeTuple(const AgentParams& value)
    {
        return std::make_tuple(value.mHeight, value.mMaxClimb, value.mRadius);
    }

    struct LessAgentParams
    {
        bool operator ()(const AgentParams& lhs, const AgentParams& rhs) const
        {
            return makeTuple(lhs) < makeTuple(rhs);
        }
    };

    using NavMeshPtr = std::unique_ptr<dtNavMesh, decltype(&dtFreeNavMesh)>;

    NavMeshPtr makeNavMesh(const AgentParams& agentParams, const RecastMesh& recastMesh)
    {
        const auto maxEdgeLen = 12;
        const auto regionMinSize = 8;
        const auto regionMergeSize = 20;
        const auto detailSampleDist = 6.0f;
        const auto detailSampleMaxError = 1.0f;

        rcContext context;
        rcConfig config;

        config.tileSize = 0;
        config.cs = 0.2f;
        config.ch = 0.2f;
        config.walkableSlopeAngle = sMaxSlope;
        config.walkableHeight = int(std::ceil(agentParams.mHeight / config.ch));
        config.walkableClimb = int(std::floor(agentParams.mMaxClimb / config.ch));
        config.walkableRadius = int(std::ceil(agentParams.mRadius / config.cs));
        config.maxEdgeLen = int(std::round(maxEdgeLen / config.cs));
        config.maxSimplificationError = 1.3f;
        config.minRegionArea = regionMinSize * regionMinSize;
        config.mergeRegionArea = regionMergeSize * regionMergeSize;
        config.maxVertsPerPoly = 6;
        config.detailSampleDist = detailSampleDist < 0.9f ? 0 : config.cs * detailSampleDist;
        config.detailSampleMaxError = config.cs * detailSampleMaxError;

        rcCalcBounds(recastMesh.getVertices().data(), int(recastMesh.getVerticesCount()), config.bmin, config.bmax);

        rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

        rcHeightfield solid;
        if (!rcCreateHeightfield(nullptr, solid, config.width, config.height, config.bmin, config.bmax,
                                 config.cs, config.ch))
            throw NavigatorException("rcCreateHeightfield failed");

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

        if (!rcRasterizeTriangles(
            &context,
            recastMesh.getVertices().data(),
            int(recastMesh.getVerticesCount()),
            recastMesh.getIndices().data(),
            areas.data(),
            int(recastMesh.getTrianglesCount()),
            solid,
            config.walkableClimb
        ))
            throw NavigatorException("rcRasterizeTriangles failed");

        rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, solid);
        rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, solid);
        rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, solid);

        rcPolyMesh polyMesh;
        rcPolyMeshDetail polyMeshDetail;
        {
            rcCompactHeightfield compact;
            compact.dist = nullptr;

            if (!rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb, solid, compact))
                throw NavigatorException("rcBuildCompactHeightfield failed");

            if (!rcErodeWalkableArea(&context, config.walkableRadius, compact))
                throw NavigatorException("rcErodeWalkableArea failed");

            if (!rcBuildDistanceField(&context, compact))
                throw NavigatorException("rcBuildDistanceField failed");

            if (!rcBuildRegions(&context, compact, 0, config.minRegionArea, config.mergeRegionArea))
                throw NavigatorException("rcBuildRegions failed");

            rcContourSet contourSet;
            if (!rcBuildContours(&context, compact, config.maxSimplificationError, config.maxEdgeLen, contourSet))
                throw NavigatorException("rcBuildContours failed");

            if (!rcBuildPolyMesh(&context, contourSet, config.maxVertsPerPoly, polyMesh))
                throw NavigatorException("rcBuildPolyMesh failed");

            if (!rcBuildPolyMeshDetail(&context, polyMesh, compact, config.detailSampleDist,
                                       config.detailSampleMaxError, polyMeshDetail))
                throw NavigatorException("rcBuildPolyMeshDetail failed");
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
        params.walkableHeight = agentParams.mHeight;
        params.walkableRadius = agentParams.mRadius;
        params.walkableClimb = agentParams.mMaxClimb;
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
        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
            throw NavigatorException("dtCreateNavMeshData failed");

        NavMeshPtr navMesh(dtAllocNavMesh(), &dtFreeNavMesh);
        if (!dtStatusSucceed(navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA)))
            throw NavigatorException("dtNavMesh::init failed");

        return navMesh;
    }

    class NavMeshManager
    {
    public:
        template <class T>
        bool addObject(T* object)
        {
            return mRecastMeshManager.addObject(object);
        }

        template <class T>
        bool removeObject(T* object)
        {
            return mRecastMeshManager.removeObject(object);
        }

        NavMeshPtr getNavMesh(const AgentParams& agentParams)
        {
            return makeNavMesh(agentParams, mRecastMeshManager.getMesh());
        }

    private:
        RecastMeshManagerCache mRecastMeshManager;
    };

    class NavMeshManagerCache
    {
    public:
        template <class T>
        bool addObject(T* object)
        {
            if (!mImpl.addObject(object))
                return false;
            mCache.clear();
            return true;
        }

        template <class T>
        bool removeObject(T* object)
        {
            if (!mImpl.removeObject(object))
                return false;
            mCache.clear();
            return true;
        }

        const dtNavMesh* getNavMesh(const AgentParams& agentParams)
        {
            auto it = mCache.find(agentParams);
            if (it == mCache.end())
                it = mCache.insert(std::make_pair(agentParams, mImpl.getNavMesh(agentParams))).first;
            return it->second.get();
        }

    private:
        NavMeshManager mImpl;
        std::map<AgentParams, NavMeshPtr, LessAgentParams> mCache;
    };

    namespace
    {

        bool inRange(const float* v1, const float* v2, const float r, const float h)
        {
            const float dx = v2[0] - v1[0];
            const float dy = v2[1] - v1[1];
            const float dz = v2[2] - v1[2];
            return (dx * dx + dz * dz) < r * r && std::abs(dy) < h;
        }

        int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
                          const dtPolyRef* visited, const int nvisited)
        {
            int furthestPath = -1;
            int furthestVisited = -1;

            // Find furthest common polygon.
            for (int i = npath-1; i >= 0; --i)
            {
                bool found = false;
                for (int j = nvisited-1; j >= 0; --j)
                {
                    if (path[i] == visited[j])
                    {
                        furthestPath = i;
                        furthestVisited = j;
                        found = true;
                    }
                }
                if (found)
                    break;
            }

            // If no intersection found just return current path.
            if (furthestPath == -1 || furthestVisited == -1)
                return npath;

            // Concatenate paths.

            // Adjust beginning of the buffer to include the visited.
            const int req = nvisited - furthestVisited;
            const int orig = rcMin(furthestPath+1, npath);
            int size = rcMax(0, npath-orig);
            if (req+size > maxPath)
                size = maxPath-req;
            if (size)
                memmove(path+req, path+orig, std::size_t(size) * sizeof(dtPolyRef));

            // Store visited
            for (int i = 0; i < req; ++i)
                path[i] = visited[(nvisited-1)-i];

            return req+size;
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
        int fixupShortcuts(dtPolyRef* path, int npath, const dtNavMeshQuery& navQuery)
        {
            if (npath < 3)
                return npath;

            // Get connected polygons
            static const int maxNeis = 16;
            dtPolyRef neis[maxNeis];
            int nneis = 0;

            const dtMeshTile* tile = 0;
            const dtPoly* poly = 0;
            if (dtStatusFailed(navQuery.getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
                return npath;

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
            static const int maxLookAhead = 6;
            int cut = 0;
            for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--)
            {
                for (int j = 0; j < nneis; j++)
                {
                    if (path[i] == neis[j])
                    {
                        cut = i;
                        break;
                    }
                }
            }
            if (cut > 1)
            {
                int offset = cut-1;
                npath -= offset;
                for (int i = 1; i < npath; i++)
                    path[i] = path[i+offset];
            }

            return npath;
        }

        bool getSteerTarget(const dtNavMeshQuery& navQuery, const float* startPos, const float* endPos,
            const float minTargetDist, const dtPolyRef* path, const int pathSize,
            float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef)
        {
            // Find steer target.
            static const int MAX_STEER_POINTS = 3;
            float steerPath[MAX_STEER_POINTS*3];
            unsigned char steerPathFlags[MAX_STEER_POINTS];
            dtPolyRef steerPathPolys[MAX_STEER_POINTS];
            int nsteerPath = 0;
            navQuery.findStraightPath(startPos, endPos, path, pathSize, steerPath, steerPathFlags,
                                      steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
            if (!nsteerPath)
                return false;

            // Find vertex far enough to steer to.
            int ns = 0;
            while (ns < nsteerPath)
            {
                // Stop at Off-Mesh link or when point is further than slop away.
                if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
                    !inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
                    break;
                ns++;
            }
            // Failed to find good point to steer to.
            if (ns >= nsteerPath)
                return false;

            dtVcopy(steerPos, &steerPath[ns*3]);
            steerPos[1] = startPos[1];
            steerPosFlag = steerPathFlags[ns];
            steerPosRef = steerPathPolys[ns];

            return true;
        }

        osg::Vec3f makeOsgVec3f(const float* values)
        {
            return osg::Vec3f(values[0], values[1], values[2]);
        }

        std::vector<osg::Vec3f> makeSmoothPath(const dtNavMesh& navMesh, const dtNavMeshQuery& navMeshQuery,
            const dtQueryFilter& filter, const osg::Vec3f& start, const osg::Vec3f& end,
            std::vector<dtPolyRef> polygonPath)
        {
            // Iterate over the path to find smooth path on the detail mesh surface.
            auto polys = polygonPath.data();
            const auto maxPolygons = int(polygonPath.size());
            auto npolys = int(polygonPath.size());
            const auto spos = start.ptr();
            const auto epos = end.ptr();

            float iterPos[3], targetPos[3];
            navMeshQuery.closestPointOnPoly(polys[0], spos, iterPos, 0);
            navMeshQuery.closestPointOnPoly(polys[npolys-1], epos, targetPos, 0);

            static const float STEP_SIZE = 0.5f;
            static const float SLOP = 0.01f;

            std::vector<osg::Vec3f> smoothPath;

            smoothPath.push_back(makeOsgVec3f(iterPos));

            // Move towards target a small advancement at a time until target reached or
            // when ran out of memory to store the path.
            while (npolys)
            {
                // Find location to steer towards.
                float steerPos[3];
                unsigned char steerPosFlag;
                dtPolyRef steerPosRef;

                if (!getSteerTarget(navMeshQuery, iterPos, targetPos, SLOP, polys, npolys, steerPos,
                                    steerPosFlag, steerPosRef))
                    break;

                bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
                bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

                // Find movement delta.
                float delta[3], len;
                dtVsub(delta, steerPos, iterPos);
                len = dtMathSqrtf(dtVdot(delta, delta));
                // If the steer target is end of path or off-mesh link, do not move past the location.
                if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
                    len = 1;
                else
                    len = STEP_SIZE / len;
                float moveTgt[3];
                dtVmad(moveTgt, iterPos, delta, len);

                // Move
                float result[3];
                dtPolyRef visited[16];
                int nvisited = 0;
                navMeshQuery.moveAlongSurface(polys[0], iterPos, moveTgt, &filter, result, visited, &nvisited, 16);

                npolys = fixupCorridor(polys, npolys, maxPolygons, visited, nvisited);
                npolys = fixupShortcuts(polys, npolys, navMeshQuery);

                float h = 0;
                navMeshQuery.getPolyHeight(polys[0], result, &h);
                result[1] = h;
                dtVcopy(iterPos, result);

                // Handle end of path and off-mesh links when close enough.
                if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
                {
                    // Reached end of path.
                    dtVcopy(iterPos, targetPos);
                    smoothPath.push_back(makeOsgVec3f(iterPos));
                    break;
                }
                else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
                {
                    // Reached off-mesh connection.
                    float startPos[3], endPos[3];

                    // Advance the path up to and over the off-mesh connection.
                    dtPolyRef prevRef = 0, polyRef = polys[0];
                    int npos = 0;
                    while (npos < npolys && polyRef != steerPosRef)
                    {
                        prevRef = polyRef;
                        polyRef = polys[npos];
                        npos++;
                    }
                    for (int i = npos; i < npolys; ++i)
                        polys[i-npos] = polys[i];
                    npolys -= npos;

                    // Handle the connection.
                    const auto status = navMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
                    if (dtStatusSucceed(status))
                    {
                        smoothPath.push_back(makeOsgVec3f(startPos));

                        // Hack to make the dotted path not visible during off-mesh connection.
                        if (smoothPath.size() & 1)
                        {
                            smoothPath.push_back(makeOsgVec3f(startPos));
                        }

                        // Move position at the other side of the off-mesh link.
                        dtVcopy(iterPos, endPos);
                        float eh = 0.0f;
                        navMeshQuery.getPolyHeight(polys[0], iterPos, &eh);
                        iterPos[1] = eh;
                    }
                }

                // Store results.
                smoothPath.push_back(makeOsgVec3f(iterPos));
            }

            return smoothPath;
        }

        std::vector<osg::Vec3f> findSmoothPath(const dtNavMesh& navMesh, osg::Vec3f halfExtents,
                                               osg::Vec3f start, osg::Vec3f end,
                                               std::size_t maxPathLength = 1024, int maxNodes = 2048)
        {
            dtNavMeshQuery navMeshQuery;
            if (!dtStatusSucceed(navMeshQuery.init(&navMesh, maxNodes)))
                throw NavigatorException("dtNavMeshQuery::init failed");

            // Detour uses second coordinate as height
            std::swap(start.y(), start.z());
            std::swap(end.y(), end.z());
            std::swap(halfExtents.y(), halfExtents.z());

            dtQueryFilter queryFilter;

            dtPolyRef startRef;
            float startPolygonPosition[3];
            if (!dtStatusSucceed(navMeshQuery.findNearestPoly(start.ptr(), halfExtents.ptr(), &queryFilter, &startRef,
                                                              startPolygonPosition)))
                throw NavigatorException("dtNavMeshQuery::findNearestPoly failed for start");

            if (startRef == 0)
                throw NavigatorException("start polygon is not found");

            dtPolyRef endRef;
            float endPolygonPosition[3];
            if (!dtStatusSucceed(navMeshQuery.findNearestPoly(end.ptr(), halfExtents.ptr(), &queryFilter, &endRef,
                                                              endPolygonPosition)))
                throw NavigatorException("dtNavMeshQuery::findNearestPoly failed for end");

            if (endRef == 0)
                throw NavigatorException("end polygon is not found");

            std::vector<dtPolyRef> polygonPath(maxPathLength);
            int pathLen;
            if (!dtStatusSucceed(navMeshQuery.findPath(startRef, endRef, start.ptr(), end.ptr(), &queryFilter,
                                                       polygonPath.data(), &pathLen, int(polygonPath.size()))))
                throw NavigatorException("dtNavMeshQuery::findPath failed");
            polygonPath.resize(std::size_t(pathLen));

            if (polygonPath.empty())
                return std::vector<osg::Vec3f>();

            auto result = makeSmoothPath(navMesh, navMeshQuery, queryFilter, start, end, std::move(polygonPath));

            // Swap coordinates back
            for (auto& v : result)
                std::swap(v.y(), v.z());

            return result;
        }

    }

    static std::ostream& operator <<(std::ostream& stream, const osg::Vec3f& value)
    {
        return stream << '(' << value.x() << ", " << value.y() << ", " << value.z() << ')';
    }

    class NavigatorImpl
    {
    public:
        NavigatorImpl(std::map<MWWorld::ConstPtr, Actor*>& actors)
            : mActors(actors)
        {}

        template <class T>
        bool addObject(T* object)
        {
            std::cout << "NavigatorImpl::addObject object=" << object << std::endl;
            return mNavMeshManager.addObject(object);
        }

        template <class T>
        bool removeObject(T* object)
        {
            std::cout << "NavigatorImpl::removeObject object=" << object << std::endl;
            return mNavMeshManager.removeObject(object);
        }

        std::vector<osg::Vec3f> findPath(const MWWorld::ConstPtr& actorPtr,
                                         const osg::Vec3f& start, const osg::Vec3f& end)
        {
            std::cout << "NavigatorImpl::findPath actor=" << actorPtr.getBase()
                      << " start=" << start << " end=" << end << std::endl;
            const auto& actor = *mActors.at(actorPtr);
            AgentParams agentParams;
            agentParams.mHeight = 2.0f * actor.getHalfExtents().z() * recastScaleFactor;
            agentParams.mMaxClimb = sStepSizeUp * recastScaleFactor;
            agentParams.mRadius = actor.getHalfExtents().x() * recastScaleFactor;
            const auto navMesh = mNavMeshManager.getNavMesh(agentParams);
            auto result = findSmoothPath(*navMesh, actor.getHalfExtents(), start * recastScaleFactor,
                                         end * recastScaleFactor);
            for (auto& v : result)
                v *= invertedRecastScaleFactor;
            return result;
        }

    private:
        std::map<MWWorld::ConstPtr, Actor*>& mActors;
        NavMeshManagerCache mNavMeshManager;
    };

    Navigator::Navigator(NavigatorImpl& impl)
        : mImpl(&impl)
    {}

    std::vector<osg::Vec3f> Navigator::findPath(const MWWorld::ConstPtr& actor,
                                                const osg::Vec3f& start, const osg::Vec3f& end) const
    {
        return mImpl->findPath(actor, start, end);
    }

    // ---------------------------------------------------------------

    PhysicsSystem::PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode)
        : mShapeManager(new Resource::BulletShapeManager(resourceSystem->getVFS(), resourceSystem->getSceneManager(), resourceSystem->getNifFileManager()))
        , mResourceSystem(resourceSystem)
        , mDebugDrawEnabled(false)
        , mTimeAccum(0.0f)
        , mWaterHeight(0)
        , mWaterEnabled(false)
        , mParentNode(parentNode)
        , mPhysicsDt(1.f / 60.f)
        , mNavigator(new NavigatorImpl(mActors))
    {
        mResourceSystem->addResourceManager(mShapeManager.get());

        mCollisionConfiguration = new btDefaultCollisionConfiguration();
        mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);
        mBroadphase = new btDbvtBroadphase();

        mCollisionWorld = new btCollisionWorld(mDispatcher, mBroadphase, mCollisionConfiguration);

        // Don't update AABBs of all objects every frame. Most objects in MW are static, so we don't need this.
        // Should a "static" object ever be moved, we have to update its AABB manually using DynamicsWorld::updateSingleAabb.
        mCollisionWorld->setForceUpdateAllAabbs(false);

        // Check if a user decided to override a physics system FPS
        const char* env = getenv("OPENMW_PHYSICS_FPS");
        if (env)
        {
            float physFramerate = std::atof(env);
            if (physFramerate > 0)
            {
                mPhysicsDt = 1.f / physFramerate;
                std::cerr << "Warning: physics framerate was overridden (a new value is " << physFramerate << ")."  << std::endl;
            }
        }
    }

    PhysicsSystem::~PhysicsSystem()
    {
        mResourceSystem->removeResourceManager(mShapeManager.get());

        if (mWaterCollisionObject.get())
            mCollisionWorld->removeCollisionObject(mWaterCollisionObject.get());

        for (HeightFieldMap::iterator it = mHeightFields.begin(); it != mHeightFields.end(); ++it)
        {
            mCollisionWorld->removeCollisionObject(it->second->getCollisionObject());
            delete it->second;
        }

        for (ObjectMap::iterator it = mObjects.begin(); it != mObjects.end(); ++it)
        {
            mCollisionWorld->removeCollisionObject(it->second->getCollisionObject());
            delete it->second;
        }

        for (ActorMap::iterator it = mActors.begin(); it != mActors.end(); ++it)
        {
            delete it->second;
        }

        delete mCollisionWorld;
        delete mCollisionConfiguration;
        delete mDispatcher;
        delete mBroadphase;
    }

    void PhysicsSystem::setUnrefQueue(SceneUtil::UnrefQueue *unrefQueue)
    {
        mUnrefQueue = unrefQueue;
    }

    Resource::BulletShapeManager *PhysicsSystem::getShapeManager()
    {
        return mShapeManager.get();
    }

    bool PhysicsSystem::toggleDebugRendering()
    {
        mDebugDrawEnabled = !mDebugDrawEnabled;

        if (mDebugDrawEnabled && !mDebugDrawer.get())
        {
            mDebugDrawer.reset(new MWRender::DebugDrawer(mParentNode, mCollisionWorld));
            mCollisionWorld->setDebugDrawer(mDebugDrawer.get());
            mDebugDrawer->setDebugMode(mDebugDrawEnabled);
        }
        else if (mDebugDrawer.get())
            mDebugDrawer->setDebugMode(mDebugDrawEnabled);
        return mDebugDrawEnabled;
    }

    void PhysicsSystem::markAsNonSolid(const MWWorld::ConstPtr &ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr);
        if (found == mObjects.end())
            return;

        found->second->setSolid(false);
    }

    bool PhysicsSystem::isOnSolidGround (const MWWorld::Ptr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (!physactor || !physactor->getOnGround())
            return false;

        CollisionMap::const_iterator found = mStandingCollisions.find(actor);
        if (found == mStandingCollisions.end())
            return true; // assume standing on terrain (which is a non-object, so not collision tracked)

        ObjectMap::const_iterator foundObj = mObjects.find(found->second);
        if (foundObj == mObjects.end())
            return false;

        if (!foundObj->second->isSolid())
            return false;

        return true;
    }

    class DeepestNotMeContactTestResultCallback : public btCollisionWorld::ContactResultCallback
    {
        const btCollisionObject* mMe;
        const std::vector<const btCollisionObject*> mTargets;

        // Store the real origin, since the shape's origin is its center
        btVector3 mOrigin;

    public:
        const btCollisionObject *mObject;
        btVector3 mContactPoint;
        btScalar mLeastDistSqr;

        DeepestNotMeContactTestResultCallback(const btCollisionObject* me, const std::vector<const btCollisionObject*>& targets, const btVector3 &origin)
          : mMe(me), mTargets(targets), mOrigin(origin), mObject(NULL), mContactPoint(0,0,0),
            mLeastDistSqr(std::numeric_limits<float>::max())
        { }

        virtual btScalar addSingleResult(btManifoldPoint& cp,
                                         const btCollisionObjectWrapper* col0Wrap,int partId0,int index0,
                                         const btCollisionObjectWrapper* col1Wrap,int partId1,int index1)
        {
            const btCollisionObject* collisionObject = col1Wrap->m_collisionObject;
            if (collisionObject != mMe)
            {
                if (!mTargets.empty())
                {
                    if ((std::find(mTargets.begin(), mTargets.end(), collisionObject) == mTargets.end()))
                    {
                        PtrHolder* holder = static_cast<PtrHolder*>(collisionObject->getUserPointer());
                        if (holder && !holder->getPtr().isEmpty() && holder->getPtr().getClass().isActor())
                            return 0.f;
                    }
                }

                btScalar distsqr = mOrigin.distance2(cp.getPositionWorldOnA());
                if(!mObject || distsqr < mLeastDistSqr)
                {
                    mObject = collisionObject;
                    mLeastDistSqr = distsqr;
                    mContactPoint = cp.getPositionWorldOnA();
                }
            }

            return 0.f;
        }
    };

    std::pair<MWWorld::Ptr, osg::Vec3f> PhysicsSystem::getHitContact(const MWWorld::ConstPtr& actor,
                                                                     const osg::Vec3f &origin,
                                                                     const osg::Quat &orient,
                                                                     float queryDistance, std::vector<MWWorld::Ptr> targets)
    {
        const MWWorld::Store<ESM::GameSetting> &store = MWBase::Environment::get().getWorld()->getStore().get<ESM::GameSetting>();

        btConeShape shape (osg::DegreesToRadians(store.find("fCombatAngleXY")->getFloat()/2.0f), queryDistance);
        shape.setLocalScaling(btVector3(1, 1, osg::DegreesToRadians(store.find("fCombatAngleZ")->getFloat()/2.0f) /
                                              shape.getRadius()));

        // The shape origin is its center, so we have to move it forward by half the length. The
        // real origin will be provided to getFilteredContact to find the closest.
        osg::Vec3f center = origin + (orient * osg::Vec3f(0.0f, queryDistance*0.5f, 0.0f));

        btCollisionObject object;
        object.setCollisionShape(&shape);
        object.setWorldTransform(btTransform(toBullet(orient), toBullet(center)));

        const btCollisionObject* me = NULL;
        std::vector<const btCollisionObject*> targetCollisionObjects;

        const Actor* physactor = getActor(actor);
        if (physactor)
            me = physactor->getCollisionObject();

        if (!targets.empty())
        {
            for (std::vector<MWWorld::Ptr>::const_iterator it = targets.begin(); it != targets.end(); ++it)
            {
                const Actor* physactor2 = getActor(*it);
                if (physactor2)
                    targetCollisionObjects.push_back(physactor2->getCollisionObject());
            }
        }

        DeepestNotMeContactTestResultCallback resultCallback(me, targetCollisionObjects, toBullet(origin));
        resultCallback.m_collisionFilterGroup = CollisionType_Actor;
        resultCallback.m_collisionFilterMask = CollisionType_World | CollisionType_Door | CollisionType_HeightMap | CollisionType_Actor;
        mCollisionWorld->contactTest(&object, resultCallback);

        if (resultCallback.mObject)
        {
            PtrHolder* holder = static_cast<PtrHolder*>(resultCallback.mObject->getUserPointer());
            if (holder)
                return std::make_pair(holder->getPtr(), toOsg(resultCallback.mContactPoint));
        }
        return std::make_pair(MWWorld::Ptr(), osg::Vec3f());
    }

    float PhysicsSystem::getHitDistance(const osg::Vec3f &point, const MWWorld::ConstPtr &target) const
    {
        btCollisionObject* targetCollisionObj = NULL;
        const Actor* actor = getActor(target);
        if (actor)
            targetCollisionObj = actor->getCollisionObject();
        if (!targetCollisionObj)
            return 0.f;

        btTransform rayFrom;
        rayFrom.setIdentity();
        rayFrom.setOrigin(toBullet(point));

        // target the collision object's world origin, this should be the center of the collision object
        btTransform rayTo;
        rayTo.setIdentity();
        rayTo.setOrigin(targetCollisionObj->getWorldTransform().getOrigin());

        btCollisionWorld::ClosestRayResultCallback cb(rayFrom.getOrigin(), rayTo.getOrigin());

        btCollisionWorld::rayTestSingle(rayFrom, rayTo, targetCollisionObj, targetCollisionObj->getCollisionShape(), targetCollisionObj->getWorldTransform(), cb);
        if (!cb.hasHit())
        {
            // didn't hit the target. this could happen if point is already inside the collision box
            return 0.f;
        }
        else
            return (point - toOsg(cb.m_hitPointWorld)).length();
    }

    class ClosestNotMeRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
    {
    public:
        ClosestNotMeRayResultCallback(const btCollisionObject* me, const std::vector<const btCollisionObject*>& targets, const btVector3& from, const btVector3& to)
            : btCollisionWorld::ClosestRayResultCallback(from, to)
            , mMe(me), mTargets(targets)
        {
        }

        virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
        {
            if (rayResult.m_collisionObject == mMe)
                return 1.f;
            if (!mTargets.empty())
            {
                if ((std::find(mTargets.begin(), mTargets.end(), rayResult.m_collisionObject) == mTargets.end()))
                {
                    PtrHolder* holder = static_cast<PtrHolder*>(rayResult.m_collisionObject->getUserPointer());
                    if (holder && !holder->getPtr().isEmpty() && holder->getPtr().getClass().isActor())
                        return 1.f;
                }
            }
            return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
        }
    private:
        const btCollisionObject* mMe;
        const std::vector<const btCollisionObject*> mTargets;
    };

    PhysicsSystem::RayResult PhysicsSystem::castRay(const osg::Vec3f &from, const osg::Vec3f &to, const MWWorld::ConstPtr& ignore, std::vector<MWWorld::Ptr> targets, int mask, int group) const
    {
        btVector3 btFrom = toBullet(from);
        btVector3 btTo = toBullet(to);

        const btCollisionObject* me = NULL;
        std::vector<const btCollisionObject*> targetCollisionObjects;

        if (!ignore.isEmpty())
        {
            const Actor* actor = getActor(ignore);
            if (actor)
                me = actor->getCollisionObject();
            else
            {
                const Object* object = getObject(ignore);
                if (object)
                    me = object->getCollisionObject();
            }
        }

        if (!targets.empty())
        {
            for (std::vector<MWWorld::Ptr>::const_iterator it = targets.begin(); it != targets.end(); ++it)
            {
                const Actor* actor = getActor(*it);
                if (actor)
                    targetCollisionObjects.push_back(actor->getCollisionObject());
            }
        }

        ClosestNotMeRayResultCallback resultCallback(me, targetCollisionObjects, btFrom, btTo);
        resultCallback.m_collisionFilterGroup = group;
        resultCallback.m_collisionFilterMask = mask;

        mCollisionWorld->rayTest(btFrom, btTo, resultCallback);

        RayResult result;
        result.mHit = resultCallback.hasHit();
        if (resultCallback.hasHit())
        {
            result.mHitPos = toOsg(resultCallback.m_hitPointWorld);
            result.mHitNormal = toOsg(resultCallback.m_hitNormalWorld);
            if (PtrHolder* ptrHolder = static_cast<PtrHolder*>(resultCallback.m_collisionObject->getUserPointer()))
                result.mHitObject = ptrHolder->getPtr();
        }
        return result;
    }

    PhysicsSystem::RayResult PhysicsSystem::castSphere(const osg::Vec3f &from, const osg::Vec3f &to, float radius)
    {
        btCollisionWorld::ClosestConvexResultCallback callback(toBullet(from), toBullet(to));
        callback.m_collisionFilterGroup = 0xff;
        callback.m_collisionFilterMask = CollisionType_World|CollisionType_HeightMap|CollisionType_Door;

        btSphereShape shape(radius);
        const btQuaternion btrot = btQuaternion::getIdentity();

        btTransform from_ (btrot, toBullet(from));
        btTransform to_ (btrot, toBullet(to));

        mCollisionWorld->convexSweepTest(&shape, from_, to_, callback);

        RayResult result;
        result.mHit = callback.hasHit();
        if (result.mHit)
        {
            result.mHitPos = toOsg(callback.m_hitPointWorld);
            result.mHitNormal = toOsg(callback.m_hitNormalWorld);
        }
        return result;
    }

    bool PhysicsSystem::getLineOfSight(const MWWorld::ConstPtr &actor1, const MWWorld::ConstPtr &actor2) const
    {
        const Actor* physactor1 = getActor(actor1);
        const Actor* physactor2 = getActor(actor2);

        if (!physactor1 || !physactor2)
            return false;

        osg::Vec3f pos1 (physactor1->getCollisionObjectPosition() + osg::Vec3f(0,0,physactor1->getHalfExtents().z() * 0.9)); // eye level
        osg::Vec3f pos2 (physactor2->getCollisionObjectPosition() + osg::Vec3f(0,0,physactor2->getHalfExtents().z() * 0.9));

        RayResult result = castRay(pos1, pos2, MWWorld::ConstPtr(), std::vector<MWWorld::Ptr>(), CollisionType_World|CollisionType_HeightMap|CollisionType_Door);

        return !result.mHit;
    }

    bool PhysicsSystem::isOnGround(const MWWorld::Ptr &actor)
    {
        Actor* physactor = getActor(actor);
        return physactor && physactor->getOnGround();
    }

    bool PhysicsSystem::canMoveToWaterSurface(const MWWorld::ConstPtr &actor, const float waterlevel)
    {
        const Actor* physicActor = getActor(actor);
        if (!physicActor)
            return false;
        const float halfZ = physicActor->getHalfExtents().z();
        const osg::Vec3f actorPosition = physicActor->getPosition();
        const osg::Vec3f startingPosition(actorPosition.x(), actorPosition.y(), actorPosition.z() + halfZ);
        const osg::Vec3f destinationPosition(actorPosition.x(), actorPosition.y(), waterlevel + halfZ);
        ActorTracer tracer;
        tracer.doTrace(physicActor->getCollisionObject(), startingPosition, destinationPosition, mCollisionWorld);
        return (tracer.mFraction >= 1.0f);
    }

    osg::Vec3f PhysicsSystem::getHalfExtents(const MWWorld::ConstPtr &actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getRenderingHalfExtents(const MWWorld::ConstPtr &actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getRenderingHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getCollisionObjectPosition(const MWWorld::ConstPtr &actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getCollisionObjectPosition();
        else
            return osg::Vec3f();
    }

    class ContactTestResultCallback : public btCollisionWorld::ContactResultCallback
    {
    public:
        ContactTestResultCallback(const btCollisionObject* testedAgainst)
            : mTestedAgainst(testedAgainst)
        {
        }

        const btCollisionObject* mTestedAgainst;

        std::vector<MWWorld::Ptr> mResult;

        virtual btScalar addSingleResult(btManifoldPoint& cp,
                                         const btCollisionObjectWrapper* col0Wrap,int partId0,int index0,
                                         const btCollisionObjectWrapper* col1Wrap,int partId1,int index1)
        {
            const btCollisionObject* collisionObject = col0Wrap->m_collisionObject;
            if (collisionObject == mTestedAgainst)
                collisionObject = col1Wrap->m_collisionObject;
            PtrHolder* holder = static_cast<PtrHolder*>(collisionObject->getUserPointer());
            if (holder)
                mResult.push_back(holder->getPtr());
            return 0.f;
        }
    };

    std::vector<MWWorld::Ptr> PhysicsSystem::getCollisions(const MWWorld::ConstPtr &ptr, int collisionGroup, int collisionMask) const
    {
        btCollisionObject* me = NULL;

        ObjectMap::const_iterator found = mObjects.find(ptr);
        if (found != mObjects.end())
            me = found->second->getCollisionObject();
        else
            return std::vector<MWWorld::Ptr>();

        ContactTestResultCallback resultCallback (me);
        resultCallback.m_collisionFilterGroup = collisionGroup;
        resultCallback.m_collisionFilterMask = collisionMask;
        mCollisionWorld->contactTest(me, resultCallback);
        return resultCallback.mResult;
    }

    osg::Vec3f PhysicsSystem::traceDown(const MWWorld::Ptr &ptr, const osg::Vec3f& position, float maxHeight)
    {
        ActorMap::iterator found = mActors.find(ptr);
        if (found ==  mActors.end())
            return ptr.getRefData().getPosition().asVec3();
        else
            return MovementSolver::traceDown(ptr, position, found->second, mCollisionWorld, maxHeight);
    }

    void PhysicsSystem::addHeightField (const float* heights, int x, int y, float triSize, float sqrtVerts, float minH, float maxH, const osg::Object* holdObject)
    {
        HeightField *heightfield = new HeightField(heights, x, y, triSize, sqrtVerts, minH, maxH, holdObject);
        mHeightFields[std::make_pair(x,y)] = heightfield;

        mCollisionWorld->addCollisionObject(heightfield->getCollisionObject(), CollisionType_HeightMap,
            CollisionType_Actor|CollisionType_Projectile);

        mNavigator->addObject(heightfield);
    }

    void PhysicsSystem::removeHeightField (int x, int y)
    {
        HeightFieldMap::iterator heightfield = mHeightFields.find(std::make_pair(x,y));
        if(heightfield != mHeightFields.end())
        {
            mNavigator->removeObject(heightfield->second);
            mCollisionWorld->removeCollisionObject(heightfield->second->getCollisionObject());
            delete heightfield->second;
            mHeightFields.erase(heightfield);
        }
    }

    void PhysicsSystem::addObject (const MWWorld::Ptr& ptr, const std::string& mesh, int collisionType)
    {
        osg::ref_ptr<Resource::BulletShapeInstance> shapeInstance = mShapeManager->getInstance(mesh);
        if (!shapeInstance || !shapeInstance->getCollisionShape())
            return;

        Object *obj = new Object(ptr, shapeInstance);
        mObjects.insert(std::make_pair(ptr, obj));

        if (obj->isAnimated())
            mAnimatedObjects.insert(obj);

        mCollisionWorld->addCollisionObject(obj->getCollisionObject(), collisionType,
                                           CollisionType_Actor|CollisionType_HeightMap|CollisionType_Projectile);

        mNavigator->addObject(obj);
    }

    void PhysicsSystem::remove(const MWWorld::Ptr &ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr);
        if (found != mObjects.end())
        {
            mNavigator->removeObject(found->second);
            mCollisionWorld->removeCollisionObject(found->second->getCollisionObject());

            if (mUnrefQueue.get())
                mUnrefQueue->push(found->second->getShapeInstance());

            mAnimatedObjects.erase(found->second);

            delete found->second;
            mObjects.erase(found);
        }

        ActorMap::iterator foundActor = mActors.find(ptr);
        if (foundActor != mActors.end())
        {
            delete foundActor->second;
            mActors.erase(foundActor);
        }
    }

    void PhysicsSystem::updateCollisionMapPtr(CollisionMap& map, const MWWorld::Ptr &old, const MWWorld::Ptr &updated)
    {
        CollisionMap::iterator found = map.find(old);
        if (found != map.end())
        {
            map[updated] = found->second;
            map.erase(found);
        }

        for (CollisionMap::iterator it = map.begin(); it != map.end(); ++it)
        {
            if (it->second == old)
                it->second = updated;
        }
    }

    void PhysicsSystem::updatePtr(const MWWorld::Ptr &old, const MWWorld::Ptr &updated)
    {
        ObjectMap::iterator found = mObjects.find(old);
        if (found != mObjects.end())
        {
            Object* obj = found->second;
            obj->updatePtr(updated);
            mObjects.erase(found);
            mObjects.insert(std::make_pair(updated, obj));
        }

        ActorMap::iterator foundActor = mActors.find(old);
        if (foundActor != mActors.end())
        {
            Actor* actor = foundActor->second;
            actor->updatePtr(updated);
            mActors.erase(foundActor);
            mActors.insert(std::make_pair(updated, actor));
        }

        updateCollisionMapPtr(mStandingCollisions, old, updated);
    }

    Actor *PhysicsSystem::getActor(const MWWorld::Ptr &ptr)
    {
        ActorMap::iterator found = mActors.find(ptr);
        if (found != mActors.end())
            return found->second;
        return NULL;
    }

    const Actor *PhysicsSystem::getActor(const MWWorld::ConstPtr &ptr) const
    {
        ActorMap::const_iterator found = mActors.find(ptr);
        if (found != mActors.end())
            return found->second;
        return NULL;
    }

    const Object* PhysicsSystem::getObject(const MWWorld::ConstPtr &ptr) const
    {
        ObjectMap::const_iterator found = mObjects.find(ptr);
        if (found != mObjects.end())
            return found->second;
        return NULL;
    }

    void PhysicsSystem::updateScale(const MWWorld::Ptr &ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr);
        if (found != mObjects.end())
        {
            float scale = ptr.getCellRef().getScale();
            found->second->setScale(scale);
            mCollisionWorld->updateSingleAabb(found->second->getCollisionObject());
            return;
        }
        ActorMap::iterator foundActor = mActors.find(ptr);
        if (foundActor != mActors.end())
        {
            foundActor->second->updateScale();
            mCollisionWorld->updateSingleAabb(foundActor->second->getCollisionObject());
            return;
        }
    }

    void PhysicsSystem::updateRotation(const MWWorld::Ptr &ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr);
        if (found != mObjects.end())
        {
            found->second->setRotation(toBullet(ptr.getRefData().getBaseNode()->getAttitude()));
            mCollisionWorld->updateSingleAabb(found->second->getCollisionObject());
            return;
        }
        ActorMap::iterator foundActor = mActors.find(ptr);
        if (foundActor != mActors.end())
        {
            if (!foundActor->second->isRotationallyInvariant())
            {
                foundActor->second->updateRotation();
                mCollisionWorld->updateSingleAabb(foundActor->second->getCollisionObject());
            }
            return;
        }
    }

    void PhysicsSystem::updatePosition(const MWWorld::Ptr &ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr);
        if (found != mObjects.end())
        {
            found->second->setOrigin(toBullet(ptr.getRefData().getPosition().asVec3()));
            mCollisionWorld->updateSingleAabb(found->second->getCollisionObject());
            return;
        }
        ActorMap::iterator foundActor = mActors.find(ptr);
        if (foundActor != mActors.end())
        {
            foundActor->second->updatePosition();
            mCollisionWorld->updateSingleAabb(foundActor->second->getCollisionObject());
            return;
        }
    }

    void PhysicsSystem::addActor (const MWWorld::Ptr& ptr, const std::string& mesh) {
        osg::ref_ptr<const Resource::BulletShape> shape = mShapeManager->getShape(mesh);
        if (!shape)
            return;

        Actor* actor = new Actor(ptr, shape, mCollisionWorld);
        mActors.insert(std::make_pair(ptr, actor));
    }

    bool PhysicsSystem::toggleCollisionMode()
    {
        ActorMap::iterator found = mActors.find(MWMechanics::getPlayer());
        if (found != mActors.end())
        {
            bool cmode = found->second->getCollisionMode();
            cmode = !cmode;
            found->second->enableCollisionMode(cmode);
            found->second->enableCollisionBody(cmode);
            return cmode;
        }

        return false;
    }

    void PhysicsSystem::queueObjectMovement(const MWWorld::Ptr &ptr, const osg::Vec3f &movement)
    {
        PtrVelocityList::iterator iter = mMovementQueue.begin();
        for(;iter != mMovementQueue.end();++iter)
        {
            if(iter->first == ptr)
            {
                iter->second = movement;
                return;
            }
        }

        mMovementQueue.push_back(std::make_pair(ptr, movement));
    }

    void PhysicsSystem::clearQueuedMovement()
    {
        mMovementQueue.clear();
        mStandingCollisions.clear();
    }

    const PtrVelocityList& PhysicsSystem::applyQueuedMovement(float dt)
    {
        mMovementResults.clear();

        mTimeAccum += dt;

        const int maxAllowedSteps = 20;
        int numSteps = mTimeAccum / (mPhysicsDt);
        numSteps = std::min(numSteps, maxAllowedSteps);

        mTimeAccum -= numSteps * mPhysicsDt;

        if (numSteps)
        {
            // Collision events should be available on every frame
            mStandingCollisions.clear();
        }

        const MWBase::World *world = MWBase::Environment::get().getWorld();
        PtrVelocityList::iterator iter = mMovementQueue.begin();
        for(;iter != mMovementQueue.end();++iter)
        {
            ActorMap::iterator foundActor = mActors.find(iter->first);
            if (foundActor == mActors.end()) // actor was already removed from the scene
                continue;
            Actor* physicActor = foundActor->second;

            float waterlevel = -std::numeric_limits<float>::max();
            const MWWorld::CellStore *cell = iter->first.getCell();
            if(cell->getCell()->hasWater())
                waterlevel = cell->getWaterLevel();

            const MWMechanics::MagicEffects& effects = iter->first.getClass().getCreatureStats(iter->first).getMagicEffects();

            bool waterCollision = false;
            if (cell->getCell()->hasWater() && effects.get(ESM::MagicEffect::WaterWalking).getMagnitude())
            {
                if (!world->isUnderwater(iter->first.getCell(), osg::Vec3f(iter->first.getRefData().getPosition().asVec3())))
                    waterCollision = true;
                else if (physicActor->getCollisionMode() && canMoveToWaterSurface(iter->first, waterlevel))
                {
                    const osg::Vec3f actorPosition = physicActor->getPosition();
                    physicActor->setPosition(osg::Vec3f(actorPosition.x(), actorPosition.y(), waterlevel));
                    waterCollision = true;
                }
            }
            physicActor->setCanWaterWalk(waterCollision);

            // Slow fall reduces fall speed by a factor of (effect magnitude / 200)
            float slowFall = 1.f - std::max(0.f, std::min(1.f, effects.get(ESM::MagicEffect::SlowFall).getMagnitude() * 0.005f));

            bool flying = world->isFlying(iter->first);

            bool wasOnGround = physicActor->getOnGround();
            osg::Vec3f position = physicActor->getPosition();
            float oldHeight = position.z();
            bool positionChanged = false;
            for (int i=0; i<numSteps; ++i)
            {
                position = MovementSolver::move(position, physicActor->getPtr(), physicActor, iter->second, mPhysicsDt,
                                                flying, waterlevel, slowFall, mCollisionWorld, mStandingCollisions);
                if (position != physicActor->getPosition())
                    positionChanged = true;
                physicActor->setPosition(position); // always set even if unchanged to make sure interpolation is correct
            }
            if (positionChanged)
                mCollisionWorld->updateSingleAabb(physicActor->getCollisionObject());

            float interpolationFactor = mTimeAccum / mPhysicsDt;
            osg::Vec3f interpolated = position * interpolationFactor + physicActor->getPreviousPosition() * (1.f - interpolationFactor);

            float heightDiff = position.z() - oldHeight;

            MWMechanics::CreatureStats& stats = iter->first.getClass().getCreatureStats(iter->first);
            if ((wasOnGround && physicActor->getOnGround()) || flying || world->isSwimming(iter->first) || slowFall < 1)
                stats.land();
            else if (heightDiff < 0)
                stats.addToFallHeight(-heightDiff);

            mMovementResults.push_back(std::make_pair(iter->first, interpolated));
        }

        mMovementQueue.clear();

        return mMovementResults;
    }

    void PhysicsSystem::stepSimulation(float dt)
    {
        for (std::set<Object*>::iterator it = mAnimatedObjects.begin(); it != mAnimatedObjects.end(); ++it)
            (*it)->animateCollisionShapes(mCollisionWorld);

#ifndef BT_NO_PROFILE
        CProfileManager::Reset();
        CProfileManager::Increment_Frame_Counter();
#endif
    }

    void PhysicsSystem::debugDraw()
    {
        if (mDebugDrawer.get())
            mDebugDrawer->step();
    }

    bool PhysicsSystem::isActorStandingOn(const MWWorld::Ptr &actor, const MWWorld::ConstPtr &object) const
    {
        for (CollisionMap::const_iterator it = mStandingCollisions.begin(); it != mStandingCollisions.end(); ++it)
        {
            if (it->first == actor && it->second == object)
                return true;
        }
        return false;
    }

    void PhysicsSystem::getActorsStandingOn(const MWWorld::ConstPtr &object, std::vector<MWWorld::Ptr> &out) const
    {
        for (CollisionMap::const_iterator it = mStandingCollisions.begin(); it != mStandingCollisions.end(); ++it)
        {
            if (it->second == object)
                out.push_back(it->first);
        }
    }

    bool PhysicsSystem::isActorCollidingWith(const MWWorld::Ptr &actor, const MWWorld::ConstPtr &object) const
    {
        std::vector<MWWorld::Ptr> collisions = getCollisions(object, CollisionType_World, CollisionType_Actor);
        return (std::find(collisions.begin(), collisions.end(), actor) != collisions.end());
    }

    void PhysicsSystem::getActorsCollidingWith(const MWWorld::ConstPtr &object, std::vector<MWWorld::Ptr> &out) const
    {
        std::vector<MWWorld::Ptr> collisions = getCollisions(object, CollisionType_World, CollisionType_Actor);
        out.insert(out.end(), collisions.begin(), collisions.end());
    }

    void PhysicsSystem::disableWater()
    {
        if (mWaterEnabled)
        {
            mWaterEnabled = false;
            updateWater();
        }
    }

    void PhysicsSystem::enableWater(float height)
    {
        if (!mWaterEnabled || mWaterHeight != height)
        {
            mWaterEnabled = true;
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::setWaterHeight(float height)
    {
        if (mWaterHeight != height)
        {
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::updateWater()
    {
        if (mWaterCollisionObject.get())
        {
            mCollisionWorld->removeCollisionObject(mWaterCollisionObject.get());
        }

        if (!mWaterEnabled)
        {
            mWaterCollisionObject.reset();
            return;
        }

        mWaterCollisionObject.reset(new btCollisionObject());
        mWaterCollisionShape.reset(new btStaticPlaneShape(btVector3(0,0,1), mWaterHeight));
        mWaterCollisionObject->setCollisionShape(mWaterCollisionShape.get());
        mCollisionWorld->addCollisionObject(mWaterCollisionObject.get(), CollisionType_Water,
                                                    CollisionType_Actor);
    }

    Navigator PhysicsSystem::getNavigator() const
    {
        return Navigator(*mNavigator);
    }
}
