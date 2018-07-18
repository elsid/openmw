#include "recastmeshmanager.hpp"
#include "debug.hpp"

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

namespace DetourNavigator
{
    RecastMeshManager::RecastMeshManager(const Settings& settings, const TileBounds& bounds)
        : mMeshBuilder(settings, bounds)
    {
    }

    bool RecastMeshManager::addObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                                      const AreaType areaType)
    {
        if (!mObjects.insert(std::make_pair(id, Object {&shape, transform, areaType})).second)
            return false;
        if (shape.isCompound())
        {
            const auto& compound = static_cast<const btCompoundShape&>(shape);
            bool result = false;
            for (int i = 0, num = compound.getNumChildShapes(); i < num; ++i)
                result = addObject(std::size_t(compound.getChildShape(i)), *compound.getChildShape(i),
                                   compound.getChildTransform(i), areaType) || result;
            return result;
        }
        else
        {
            rebuild();
            return mMeshBuilder.addObject(shape, transform, areaType);
        }
    }

    bool RecastMeshManager::updateObject(std::size_t id, const btCollisionShape& shape, const btTransform& transform,
                                         const AreaType areaType)
    {
        const auto object = mObjects.find(id);
        if (object == mObjects.end())
            return false;
        bool childChanged = false;
        if (shape.isCompound())
        {
            const auto& compound = static_cast<const btCompoundShape&>(shape);
            for (int i = 0, num = compound.getNumChildShapes(); i < num; ++i)
                childChanged = updateObject(std::size_t(compound.getChildShape(i)), *compound.getChildShape(i),
                                      compound.getChildTransform(i), areaType) || childChanged;
        }
        const bool shapeChanged = object->second.mShape != &shape;
        const bool transformChanged = !(object->second.mTransform == transform);
        if (!shapeChanged && !transformChanged && !childChanged)
            return false;
        if (shapeChanged)
            object->second.mShape = &shape;
        if (transformChanged)
            object->second.mTransform = transform;
        mShouldRebuild = true;
        return true;
    }

    boost::optional<RecastMeshManager::Object> RecastMeshManager::removeObject(std::size_t id)
    {
        const auto object = mObjects.find(id);
        if (object == mObjects.end())
            return boost::none;
        const auto result = object->second;
        mObjects.erase(object);
        if (result.mShape->isCompound())
        {
            bool removedChildShape = false;
            const auto& compound = static_cast<const btCompoundShape&>(*result.mShape);
            for (int i = 0, num = compound.getNumChildShapes(); i < num; ++i)
                removedChildShape = removeObject(std::size_t(compound.getChildShape(i))) || removedChildShape;
            return removedChildShape ? boost::optional<RecastMeshManager::Object>(result) : boost::none;
        }
        else
        {
            mShouldRebuild = true;
            return result;
        }
    }

    std::shared_ptr<RecastMesh> RecastMeshManager::getMesh()
    {
        rebuild();
        return mMeshBuilder.create();
    }

    bool RecastMeshManager::isEmpty() const
    {
        return mObjects.empty();
    }

    void RecastMeshManager::rebuild()
    {
        if (mShouldRebuild)
        {
            mMeshBuilder.reset();
            for (const auto& v : mObjects)
                mMeshBuilder.addObject(*v.second.mShape, v.second.mTransform, v.second.mAreaType);
            mShouldRebuild = false;
        }
    }
}
