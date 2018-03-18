#include "cellpreloader.hpp"

#include <iostream>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

#include <components/resource/scenemanager.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/bulletshapemanager.hpp>
#include <components/resource/keyframemanager.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/stringops.hpp>
#include <components/nifosg/nifloader.hpp>
#include <components/terrain/world.hpp>
#include <components/esmterrain/storage.hpp>
#include <components/sceneutil/unrefqueue.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/esm/loadcell.hpp>
#include <components/detournavigator/navigator.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwrender/landmanager.hpp"

#include "../mwphysics/actor.hpp"
#include "../mwphysics/heightfield.hpp"
#include "../mwphysics/object.hpp"
#include "../mwphysics/convert.hpp"

#include "cellstore.hpp"
#include "manualref.hpp"
#include "class.hpp"

namespace MWWorld
{

    struct ListModelsVisitor
    {
        ListModelsVisitor(std::vector<std::string>& out)
            : mOut(out)
        {
        }

        virtual bool operator()(const MWWorld::Ptr& ptr)
        {
            ptr.getClass().getModelsToPreload(ptr, mOut);

            return true;
        }

        std::vector<std::string>& mOut;
    };

    /// Worker thread item: preload models in a cell.
    class PreloadItem : public SceneUtil::WorkItem
    {
    public:
        /// Constructor to be called from the main thread.
        PreloadItem(MWWorld::CellStore* cell, Resource::SceneManager* sceneManager,
                Resource::BulletShapeManager* bulletShapeManager, Resource::KeyframeManager* keyframeManager,
                Terrain::World* terrain, MWRender::LandManager* landManager, bool preloadInstances,
                const VFS::Manager* vfs)
            : mIsExterior(cell->getCell()->isExterior())
            , mX(cell->getCell()->getGridX())
            , mY(cell->getCell()->getGridY())
            , mSceneManager(sceneManager)
            , mBulletShapeManager(bulletShapeManager)
            , mKeyframeManager(keyframeManager)
            , mTerrain(terrain)
            , mLandManager(landManager)
            , mPreloadInstances(preloadInstances)
            , mCellId(std::size_t(cell->getCell()))
            , mAbort(false)
        {
            mTerrainView = mTerrain->createView();

            ListModelsVisitor visitor (mMeshes);
            if (cell->getState() == MWWorld::CellStore::State_Loaded)
            {
                cell->forEach(visitor);
                cell->forEachConst([&] (const MWWorld::ConstPtr& ptr) {
                    const auto id = std::size_t(ptr.getBase());
                    btTransform transform;
//                    transform.setRotation(MWPhysics::toBullet(ptr.getRefData().getBaseNode()->getAttitude()));
                    transform.setOrigin(MWPhysics::toBullet(ptr.getRefData().getPosition().asVec3()));
                    mNavigatorObjects.push_back(NavigatorObject {id, getModelName(ptr, vfs), transform,
                                                                 ptr.getClass().isActor()});
                    return true;
                });
            }
            else
            {
                const std::vector<std::string>& objectIds = cell->getPreloadedIds();

                // could possibly build the model list in the worker thread if we manage to make the Store thread safe
                for (std::vector<std::string>::const_iterator it = objectIds.begin(); it != objectIds.end(); ++it)
                {
                    MWWorld::ManualRef ref(MWBase::Environment::get().getWorld()->getStore(), *it);
                    std::string model = ref.getPtr().getClass().getModel(ref.getPtr());
                    if (!model.empty())
                        mMeshes.push_back(model);
                }
            }
        }

        virtual void abort()
        {
            mAbort = true;
        }

        /// Preload work to be called from the worker thread.
        virtual void doWork()
        {
            const auto navigator = MWBase::Environment::get().getWorld()->getNavigator();

            if (mIsExterior)
            {
                try
                {
                    mTerrain->cacheCell(mTerrainView.get(), mX, mY);
                    const auto land = mLandManager->getLand(mX, mY);
                    mPreloadedObjects.push_back(land);

                    const auto verts = ESM::Land::LAND_SIZE;
                    const auto worldSize = ESM::Land::REAL_SIZE;
                    const auto triSize = worldSize / (verts - 1);

                    if (const auto data = land ? land->getData(ESM::Land::DATA_VHGT) : 0)
                    {
                        btHeightfieldTerrainShape shape(
                            verts, verts, data->mHeights, 1,
                            data->mMinHeight, data->mMaxHeight, 2,
                            PHY_FLOAT, false
                        );

                        shape.setLocalScaling(btVector3(triSize, triSize, 1));

                        const btTransform transform(
                            btQuaternion::getIdentity(),
                            btVector3(
                                (mX + 0.5f) * triSize * (verts - 1),
                                (mY + 0.5f) * triSize * (verts - 1),
                                (data->mMaxHeight + data->mMinHeight) * 0.5f
                            )
                        );

                        navigator->addObject(mCellId, shape, transform);
                    }
                    else
                    {
                        static std::vector<float> defaultHeight;
                        defaultHeight.resize(std::size_t(verts) * std::size_t(verts), ESM::Land::DEFAULT_HEIGHT);

                        btHeightfieldTerrainShape shape(
                            verts, verts, defaultHeight.data(), 1,
                            ESM::Land::DEFAULT_HEIGHT, ESM::Land::DEFAULT_HEIGHT, 2,
                            PHY_FLOAT, false
                        );

                        shape.setLocalScaling(btVector3(triSize, triSize, 1));

                        const btTransform transform(
                            btQuaternion::getIdentity(),
                            btVector3(
                                (mX + 0.5f) * triSize * (verts - 1),
                                (mY + 0.5f) * triSize * (verts - 1),
                                ESM::Land::DEFAULT_HEIGHT
                            )
                        );

                        navigator->addObject(mCellId, shape, transform);
                    }
                }
                catch(std::exception& e)
                {
                    std::cout << e.what() << std::endl;
                }
            }

            for (MeshList::const_iterator it = mMeshes.begin(); it != mMeshes.end(); ++it)
            {
                if (mAbort)
                    break;

                try
                {
                    std::string mesh  = *it;
                    mesh = Misc::ResourceHelpers::correctActorModelPath(mesh, mSceneManager->getVFS());

                    if (mPreloadInstances)
                    {
                        mPreloadedObjects.push_back(mSceneManager->cacheInstance(mesh));
                        mPreloadedObjects.push_back(mBulletShapeManager->cacheInstance(mesh));
                    }
                    else
                    {
                        mPreloadedObjects.push_back(mSceneManager->getTemplate(mesh));
                        mPreloadedObjects.push_back(mBulletShapeManager->getShape(mesh));
                    }

                    size_t slashpos = mesh.find_last_of("/\\");
                    if (slashpos != std::string::npos && slashpos != mesh.size()-1)
                    {
                        Misc::StringUtils::lowerCaseInPlace(mesh);
                        if (mesh[slashpos+1] == 'x')
                        {
                            std::string kfname = mesh;
                            if(kfname.size() > 4 && kfname.compare(kfname.size()-4, 4, ".nif") == 0)
                            {
                                kfname.replace(kfname.size()-4, 4, ".kf");
                                mPreloadedObjects.push_back(mKeyframeManager->get(kfname));
                            }

                        }
                    }
                }
                catch (std::exception& e)
                {
                    // ignore error for now, would spam the log too much
                    // error will be shown when visiting the cell
                }
            }

            for (const auto& v : mNavigatorObjects)
            {
                try
                {
                    const auto shapeInstance = mBulletShapeManager->getInstance(v.mModelName);

                    if (!shapeInstance)
                        continue;

                    if (v.mIsActor)
                    {
                        navigator->addAgent(shapeInstance->mCollisionBoxHalfExtents);
                        continue;
                    }

                    if (!shapeInstance->getCollisionShape())
                        continue;

//                    if (const auto concave = dynamic_cast<const btConcaveShape*>(shapeInstance->getCollisionShape()))
//                        navigator->addObject(v.mId, *concave, v.mTransform);
                }
                catch (const std::exception& e)
                {
                    std::cout << e.what() << std::endl;
                }
            }

            try
            {
                navigator->update();
            }
            catch (const std::exception& e)
            {
                std::cout << e.what() << std::endl;
            }
        }

    private:
        typedef std::vector<std::string> MeshList;

        struct NavigatorObject
        {
            std::size_t mId;
            std::string mModelName;
            btTransform mTransform;
            bool mIsActor;
        };

        bool mIsExterior;
        int mX;
        int mY;
        MeshList mMeshes;
        Resource::SceneManager* mSceneManager;
        Resource::BulletShapeManager* mBulletShapeManager;
        Resource::KeyframeManager* mKeyframeManager;
        Terrain::World* mTerrain;
        MWRender::LandManager* mLandManager;
        bool mPreloadInstances;
        std::size_t mCellId;

        volatile bool mAbort;

        osg::ref_ptr<Terrain::View> mTerrainView;

        // keep a ref to the loaded objects to make sure it stays loaded as long as this cell is in the preloaded state
        std::vector<osg::ref_ptr<const osg::Object> > mPreloadedObjects;

        std::vector<NavigatorObject> mNavigatorObjects;
    };

    /// Worker thread item: update the resource system's cache, effectively deleting unused entries.
    class UpdateCacheItem : public SceneUtil::WorkItem
    {
    public:
        UpdateCacheItem(Resource::ResourceSystem* resourceSystem, double referenceTime)
            : mReferenceTime(referenceTime)
            , mResourceSystem(resourceSystem)
        {
        }

        virtual void doWork()
        {
            mResourceSystem->updateCache(mReferenceTime);
        }

    private:
        double mReferenceTime;
        Resource::ResourceSystem* mResourceSystem;
    };

    CellPreloader::CellPreloader(Resource::ResourceSystem* resourceSystem, Resource::BulletShapeManager* bulletShapeManager, Terrain::World* terrain, MWRender::LandManager* landManager)
        : mResourceSystem(resourceSystem)
        , mBulletShapeManager(bulletShapeManager)
        , mTerrain(terrain)
        , mLandManager(landManager)
        , mExpiryDelay(0.0)
        , mMinCacheSize(0)
        , mMaxCacheSize(0)
        , mPreloadInstances(true)
        , mLastResourceCacheUpdate(0.0)
    {
    }

    CellPreloader::~CellPreloader()
    {
        if (mTerrainPreloadItem)
        {
            mTerrainPreloadItem->abort();
            mTerrainPreloadItem->waitTillDone();
            mTerrainPreloadItem = NULL;
        }

        if (mUpdateCacheItem)
        {
            mUpdateCacheItem->waitTillDone();
            mUpdateCacheItem = NULL;
        }

        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end();++it)
            it->second.mWorkItem->abort();

        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end();++it)
            it->second.mWorkItem->waitTillDone();

        mPreloadCells.clear();
    }

    void CellPreloader::preload(CellStore *cell, double timestamp)
    {
        if (!mWorkQueue)
        {
            std::cerr << "Error: can't preload, no work queue set " << std::endl;
            return;
        }
        if (cell->getState() == CellStore::State_Unloaded)
        {
            std::cerr << "Error: can't preload objects for unloaded cell" << std::endl;
            return;
        }

        PreloadMap::iterator found = mPreloadCells.find(cell);
        if (found != mPreloadCells.end())
        {
            // already preloaded, nothing to do other than updating the timestamp
            found->second.mTimeStamp = timestamp;
            return;
        }

        while (mPreloadCells.size() >= mMaxCacheSize)
        {
            // throw out oldest cell to make room
            PreloadMap::iterator oldestCell = mPreloadCells.begin();
            double oldestTimestamp = DBL_MAX;
            double threshold = 1.0; // seconds
            for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end(); ++it)
            {
                if (it->second.mTimeStamp < oldestTimestamp)
                {
                    oldestTimestamp = it->second.mTimeStamp;
                    oldestCell = it;
                }
            }

            if (oldestTimestamp + threshold < timestamp)
            {
                oldestCell->second.mWorkItem->abort();
                mPreloadCells.erase(oldestCell);
            }
            else
                return;
        }

        osg::ref_ptr<PreloadItem> item (new PreloadItem(cell, mResourceSystem->getSceneManager(), mBulletShapeManager,
                                                        mResourceSystem->getKeyframeManager(), mTerrain, mLandManager,
                                                        mPreloadInstances, mResourceSystem->getVFS()));
        mWorkQueue->addWorkItem(item);

        mPreloadCells[cell] = PreloadEntry(timestamp, item);
    }

    void CellPreloader::notifyLoaded(CellStore *cell)
    {
        PreloadMap::iterator found = mPreloadCells.find(cell);
        if (found != mPreloadCells.end())
        {
            // do the deletion in the background thread
            if (found->second.mWorkItem)
            {
                found->second.mWorkItem->abort();
                mUnrefQueue->push(mPreloadCells[cell].mWorkItem);
            }

            mPreloadCells.erase(found);
        }
    }

    void CellPreloader::clear()
    {
        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end();)
        {
            if (it->second.mWorkItem)
            {
                it->second.mWorkItem->abort();
                mUnrefQueue->push(it->second.mWorkItem);
            }

            mPreloadCells.erase(it++);
        }
    }

    void CellPreloader::updateCache(double timestamp)
    {
        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end();)
        {
            if (mPreloadCells.size() >= mMinCacheSize && it->second.mTimeStamp < timestamp - mExpiryDelay)
            {
                if (it->second.mWorkItem)
                {
                    it->second.mWorkItem->abort();
                    mUnrefQueue->push(it->second.mWorkItem);
                }
                mPreloadCells.erase(it++);
            }
            else
                ++it;
        }

        if (timestamp - mLastResourceCacheUpdate > 1.0 && (!mUpdateCacheItem || mUpdateCacheItem->isDone()))
        {
            // the resource cache is cleared from the worker thread so that we're not holding up the main thread with delete operations
            mUpdateCacheItem = new UpdateCacheItem(mResourceSystem, timestamp);
            mWorkQueue->addWorkItem(mUpdateCacheItem, true);
            mLastResourceCacheUpdate = timestamp;
        }
    }

    void CellPreloader::setExpiryDelay(double expiryDelay)
    {
        mExpiryDelay = expiryDelay;
    }

    void CellPreloader::setMinCacheSize(unsigned int num)
    {
        mMinCacheSize = num;
    }

    void CellPreloader::setMaxCacheSize(unsigned int num)
    {
        mMaxCacheSize = num;
    }

    void CellPreloader::setPreloadInstances(bool preload)
    {
        mPreloadInstances = preload;
    }

    unsigned int CellPreloader::getMaxCacheSize() const
    {
        return mMaxCacheSize;
    }

    void CellPreloader::setWorkQueue(osg::ref_ptr<SceneUtil::WorkQueue> workQueue)
    {
        mWorkQueue = workQueue;
    }

    void CellPreloader::setUnrefQueue(SceneUtil::UnrefQueue* unrefQueue)
    {
        mUnrefQueue = unrefQueue;
    }

    class TerrainPreloadItem : public SceneUtil::WorkItem
    {
    public:
        TerrainPreloadItem(const std::vector<osg::ref_ptr<Terrain::View> >& views, Terrain::World* world, const std::vector<osg::Vec3f>& preloadPositions)
            : mAbort(false)
            , mTerrainViews(views)
            , mWorld(world)
            , mPreloadPositions(preloadPositions)
        {
        }

        virtual void doWork()
        {
            for (unsigned int i=0; i<mTerrainViews.size() && i<mPreloadPositions.size() && !mAbort; ++i)
            {
                mWorld->preload(mTerrainViews[i], mPreloadPositions[i]);
                mTerrainViews[i]->reset(0);
            }
        }

        virtual void abort()
        {
            mAbort = true;
        }

    private:
        volatile bool mAbort;
        std::vector<osg::ref_ptr<Terrain::View> > mTerrainViews;
        Terrain::World* mWorld;
        std::vector<osg::Vec3f> mPreloadPositions;
    };

    void CellPreloader::setTerrainPreloadPositions(const std::vector<osg::Vec3f> &positions)
    {
        if (mTerrainPreloadItem && !mTerrainPreloadItem->isDone())
            return;
        else if (positions == mTerrainPreloadPositions)
            return;
        else
        {
            if (mTerrainViews.size() > positions.size())
            {
                for (unsigned int i=positions.size(); i<mTerrainViews.size(); ++i)
                    mUnrefQueue->push(mTerrainViews[i]);
                mTerrainViews.resize(positions.size());
            }
            else if (mTerrainViews.size() < positions.size())
            {
                for (unsigned int i=mTerrainViews.size(); i<positions.size(); ++i)
                    mTerrainViews.push_back(mTerrain->createView());
            }

            // TODO: provide some way of giving the preloaded view to the main thread when we enter the cell
            // right now, we just use it to make sure the resources are preloaded
            mTerrainPreloadPositions = positions;
            mTerrainPreloadItem = new TerrainPreloadItem(mTerrainViews, mTerrain, positions);
            mWorkQueue->addWorkItem(mTerrainPreloadItem);
        }
    }

    std::string getModelName(const MWWorld::ConstPtr& ptr, const VFS::Manager* vfs)
    {
        const bool useAnim = ptr.getClass().useAnim();
        std::string model = ptr.getClass().getModel(ptr);
        if (useAnim)
            model = Misc::ResourceHelpers::correctActorModelPath(model, vfs);

        const std::string id = ptr.getCellRef().getRefId();
        if (id == "prisonmarker" || id == "divinemarker" || id == "templemarker" || id == "northmarker")
            model = ""; // marker objects that have a hardcoded function in the game logic, should be hidden from the player

        return model;
    }

}
