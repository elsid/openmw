#ifndef GAME_SOUND_SOUND_BUFFER_H
#define GAME_SOUND_SOUND_BUFFER_H

#include <algorithm>
#include <string>
#include <memory>
#include <deque>
#include <unordered_map>

#include "sound_output.hpp"

#include <components/misc/objectpool.hpp>

namespace ESM
{
    struct Sound;
}

namespace VFS
{
    class Manager;
}

namespace MWSound
{
    struct SoundBufferParams
    {
        std::string mResourceName;
        float mVolume = 0;
        float mMinDist = 0;
        float mMaxDist = 0;
    };

    class SoundBufferPool;

    class Sound_Buffer
    {
        public:
            void init(SoundBufferParams params)
            {
                mParams = std::move(params);
                mHandle = nullptr;
                mUses = 0;
            }

            const std::string& getResourceName() const noexcept { return mParams.mResourceName; }

            Sound_Handle getHandle() const noexcept { return mHandle; }

            float getVolume() const noexcept { return mParams.mVolume; }

            float getMinDist() const noexcept { return mParams.mMinDist; }

            float getMaxDist() const noexcept { return mParams.mMaxDist; }

        private:
            SoundBufferParams mParams;
            Sound_Handle mHandle = nullptr;
            std::size_t mUses = 0;

            friend class SoundBufferPool;
    };

    class SoundBufferPool
    {
        public:
            SoundBufferPool(const VFS::Manager& vfs, Sound_Output& output);

            ~SoundBufferPool();

            Sound_Buffer* lookup(const std::string& soundId) const;

            Sound_Buffer* load(const std::string& soundId);

            void use(Sound_Buffer& sfx) noexcept
            {
                if (sfx.mUses++ == 0)
                {
                    const auto it = std::find(mUnusedBuffers.begin(), mUnusedBuffers.end(), &sfx);
                    if (it != mUnusedBuffers.end())
                        mUnusedBuffers.erase(it);
                }
            }

            void release(Sound_Buffer& sfx)
            {
                if (--sfx.mUses == 0)
                    mUnusedBuffers.push_front(&sfx);
            }

            void clear();

        private:
            const VFS::Manager* const mVfs;
            Sound_Output* mOutput;
            Misc::ObjectPool<Sound_Buffer> mSoundBuffers;
            std::unordered_map<std::string, Misc::ObjectPtr<Sound_Buffer>> mBufferNameMap;
            std::size_t mBufferCacheMin;
            std::size_t mBufferCacheMax;
            std::size_t mBufferCacheSize = 0;
            // NOTE: unused buffers are stored in front-newest order.
            std::deque<Sound_Buffer*> mUnusedBuffers;

            inline Sound_Buffer* insertSound(const std::string& soundId, const ESM::Sound& sound);

            inline void unloadUnused();
    };
}

#endif /* GAME_SOUND_SOUND_BUFFER_H */
