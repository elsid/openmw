#ifndef OPENMW_COMPONENTS_MISC_OBJECTPOOL_H
#define OPENMW_COMPONENTS_MISC_OBJECTPOOL_H

#include <deque>
#include <memory>
#include <vector>
#include <mutex>

namespace Misc
{
    template <class T>
    class ObjectPool;

    template <class T>
    class ObjectPtrDeleter
    {
        public:
            ObjectPtrDeleter(std::nullptr_t)
                : mPool(nullptr) {}

            ObjectPtrDeleter(ObjectPool<T>& pool)
                : mPool(&pool) {}

            void operator()(T* object) const
            {
                mPool->recycle(object);
            }

        private:
            ObjectPool<T>* mPool;
    };

    template <class T>
    class ObjectPool
    {
        friend class ObjectPtrDeleter<T>;

        public:
            ObjectPool()
                : mObjects(std::make_unique<std::deque<T>>()) {}

            std::shared_ptr<T> get()
            {
                T* object;

                {
                    std::lock_guard<std::mutex> lock(mMutex);

                    if (!mUnused.empty())
                    {
                        object = mUnused.back();
                        mUnused.pop_back();
                    }
                    else
                    {
                        mObjects->emplace_back();
                        object = &mObjects->back();
                    }
                }

                return std::shared_ptr<T>(object, ObjectPtrDeleter<T>(*this));
            }

        private:
            std::mutex mMutex;
            std::shared_ptr<std::deque<T>> mObjects;
            std::vector<T*> mUnused;

            void recycle(T* object)
            {
                std::lock_guard<std::mutex> lock(mMutex);
                mUnused.push_back(object);
            }
    };
}

#endif
