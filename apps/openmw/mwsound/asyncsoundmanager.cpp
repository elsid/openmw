#include "asyncsoundmanager.hpp"
#include "sound.hpp"

#include "../mwmechanics/actorutil.hpp"

#include <components/misc/stringops.hpp>

namespace MWSound
{
    namespace
    {
        template <class Function>
        class WorkItem final : public SceneUtil::WorkItem
        {
        public:
            WorkItem(Function&& function) : mFunction(std::move(function)) {}

            void doWork() final
            {
                mFunction();
            }

        private:
            Function mFunction;
        };

        template <class Function>
        auto makeWorkItem(Function&& function)
        {
            using WorkItem = WorkItem<std::decay_t<Function>>;
            return osg::ref_ptr<WorkItem>(new WorkItem(std::forward<Function>(function)));
        }
    }

    AsyncSoundManager::AsyncSoundManager(const VFS::Manager* vfs, bool useSound)
        : mImpl(vfs, useSound),
          mWorkQueue(1)
    {
    }

    void AsyncSoundManager::processChangedSettings(const Settings::CategorySettingVector& settings)
    {
        addWorkItem([this, settings = settings] { mImpl.processChangedSettings(settings); });
    }

    void AsyncSoundManager::stopMusic()
    {
        for (auto& v : *mLoadingMusic.lock())
            v.mCancelled = true;

        addWorkItem([this] { mImpl.stopMusic(); });
    }

    void AsyncSoundManager::streamMusic(const std::string& fileName)
    {
        std::list<LoadingMusic>::iterator it;

        {
            const auto music = mLoadingMusic.lock();
            it = music->emplace(music->end());
        }

        it->mFileName = fileName;

        addWorkItem([this, it]
        {
            if (!it->mCancelled)
                mImpl.streamMusic(it->mFileName);
            mLoadingMusic.lock()->erase(it);
        });
    }

    bool AsyncSoundManager::isMusicPlaying()
    {
        return !mLoadingMusic.lockConst()->empty() || mImpl.isMusicPlaying();
    }

    void AsyncSoundManager::playPlaylist(const std::string& playList)
    {
        std::list<LoadingMusic>::iterator it;

        {
            const auto music = mLoadingMusic.lock();
            it = music->emplace(music->end());
        }

        it->mFileName = playList;

        addWorkItem([this, it]
        {
            if (!it->mCancelled)
                mImpl.playPlaylist(it->mFileName);
            mLoadingMusic.lock()->erase(it);
        });
    }

    void AsyncSoundManager::playTitleMusic()
    {
        std::list<LoadingMusic>::iterator it;

        {
            const auto music = mLoadingMusic.lock();
            it = music->emplace(music->end());
        }

        addWorkItem([this, it]
        {
            if (!it->mCancelled)
                mImpl.playTitleMusic();
            mLoadingMusic.lock()->erase(it);
        });
    }

    void AsyncSoundManager::say(const MWWorld::ConstPtr& ptr, const std::string& fileName)
    {
        std::list<LoadingVoice>::iterator it;

        {
            const auto voices = mLoadingVoices.lock();
            it = voices->emplace(voices->end());
        }

        it->mPtr = ptr;
        it->mPlayLocal = ptr == MWMechanics::getPlayer();
        it->mFileName = fileName;
        it->mStream = mImpl.makeStream(ptr);
        it->mActive = false;

        addWorkItem([this, it]
        {
            if (!it->mCancelled)
                mImpl.saySync(it->mPtr, it->mPlayLocal, it->mFileName, std::move(it->mStream));
            mLoadingVoices.lock()->erase(it);
        });
    }

    void AsyncSoundManager::say(const std::string& fileName)
    {
        std::list<LoadingVoice>::iterator it;

        {
            const auto voices = mLoadingVoices.lock();
            it = voices->emplace(voices->end());
        }

        it->mFileName = fileName;
        it->mStream = mImpl.makeStream(MWWorld::ConstPtr());
        it->mActive = true;

        addWorkItem([this, it]
        {
            if (!it->mCancelled)
                mImpl.saySync(it->mFileName, std::move(it->mStream));
            mLoadingVoices.lock()->erase(it);
        });
    }

    bool AsyncSoundManager::sayDone(const MWWorld::ConstPtr& ptr) const
    {
        if (!mImpl.sayDone(ptr))
            return false;

        const auto hasPtr = [&] (const LoadingVoice& v) { return v.mPtr == ptr && v.mActive && !v.mCancelled; };

        const auto voices = mLoadingVoices.lockConst();
        return voices->end() == std::find_if(voices->begin(), voices->end(), hasPtr);
    }

    bool AsyncSoundManager::sayActive(const MWWorld::ConstPtr& ptr) const
    {
        if (mImpl.sayActive(ptr))
            return true;

        const auto hasPtr = [&] (const LoadingVoice& v) { return v.mPtr == ptr && !v.mCancelled; };

        const auto voices = mLoadingVoices.lockConst();
        return voices->end() != std::find_if(voices->begin(), voices->end(), hasPtr);
    }

    void AsyncSoundManager::stopSay(const MWWorld::ConstPtr& ptr)
    {
        for (auto& v : *mLoadingVoices.lock())
            if (v.mPtr == ptr)
                v.mCancelled = true;

        addWorkItem([this, ptr = ptr] { mImpl.stopSay(ptr); });
    }

    float AsyncSoundManager::getSaySoundLoudness(const MWWorld::ConstPtr& ptr) const
    {
        return mImpl.getSaySoundLoudness(ptr);
    }

    StreamRef AsyncSoundManager::playTrack(const DecoderPtr& decoder, Type type)
    {
        return mImpl.playTrack(decoder, type);
    }

    void AsyncSoundManager::stopTrack(StreamPtr stream)
    {
        addWorkItem([this, stream] { mImpl.stopTrack(stream); });
    }

    double AsyncSoundManager::getTrackTimeDelay(StreamPtr stream)
    {
        return mImpl.getTrackTimeDelay(stream);
    }

    SoundRef AsyncSoundManager::playSound(const std::string& soundId, float volume, float pitch, Type type, PlayMode mode, float offset)
    {
        return playSoundAsync(MWWorld::ConstPtr(), soundId, offset,
                              mImpl.makeSound(volume, pitch, type, mode));
    }

    SoundRef AsyncSoundManager::playSound3D(const MWWorld::ConstPtr& ptr, const std::string& soundId,
                                            float volume, float pitch, Type type, PlayMode mode, float offset)
    {
        return playSoundAsync(ptr, soundId, offset,
                              mImpl.makeSound3D(ptr, volume, pitch, type, mode));
    }

    SoundRef AsyncSoundManager::playSound3D(const osg::Vec3f& initialPos, const std::string& soundId,
                                            float volume, float pitch, Type type, PlayMode mode, float offset)
    {
        return playSoundAsync(MWWorld::ConstPtr(), soundId, offset,
                              mImpl.makeSound3D(initialPos, volume, pitch, type, mode));
    }

    void AsyncSoundManager::stopSound(SoundPtr sound)
    {
        for (auto& v : *mLoadingSounds.lock())
            if (v.mSound == sound)
                v.mCancelled = true;

        addWorkItem([this, sound] { mImpl.stopSound(sound); });
    }

    void AsyncSoundManager::stopSound3D(const MWWorld::ConstPtr& ptr, const std::string& soundId)
    {
        for (auto& v : *mLoadingSounds.lock())
            if (v.mPtr == ptr && v.mSoundId == soundId)
                v.mCancelled = true;

        addWorkItem([this, ptr = ptr, soundId = soundId] { mImpl.stopSound3D(ptr, soundId); });
    }

    void AsyncSoundManager::stopSound3D(const MWWorld::ConstPtr& ptr)
    {
        for (auto& v : *mLoadingSounds.lock())
            if (v.mPtr == ptr)
                v.mCancelled = true;

        addWorkItem([this, ptr = ptr] { mImpl.stopSound3D(ptr); });
    }

    void AsyncSoundManager::stopSound(const MWWorld::CellStore* cell)
    {
        for (auto& v : *mLoadingSounds.lock())
            if (!v.mPtr.isEmpty() && v.mPtr != MWMechanics::getPlayer() && v.mPtr.getCell() == cell)
                v.mCancelled = true;

        addWorkItem([this, cell] { mImpl.stopSound(cell); });
    }

    void AsyncSoundManager::fadeOutSound3D(const MWWorld::ConstPtr& ptr, const std::string& soundId, float duration)
    {
        addWorkItem([this, ptr = ptr, soundId = soundId, duration]
            { mImpl.fadeOutSound3D(ptr, soundId, duration); });
    }

    bool AsyncSoundManager::getSoundPlaying(const MWWorld::ConstPtr& ptr, const std::string& soundId) const
    {
        return mImpl.getSoundPlaying(ptr, soundId);
    }

    void AsyncSoundManager::pauseSounds(BlockerType blocker, int types)
    {
        addWorkItem([this, blocker, types] { mImpl.pauseSounds(blocker, types); });
    }

    void AsyncSoundManager::resumeSounds(BlockerType blocker)
    {
        addWorkItem([this, blocker] { mImpl.resumeSounds(blocker); });
    }

    void AsyncSoundManager::pausePlayback()
    {
        addWorkItem([this] { mImpl.pausePlayback(); });
    }

    void AsyncSoundManager::resumePlayback()
    {
        addWorkItem([this] { mImpl.resumePlayback(); });
    }

    void AsyncSoundManager::update(float duration)
    {
        addWorkItem([this, duration] { mImpl.update(duration); });
    }

    void AsyncSoundManager::setListenerPosDir(const osg::Vec3f& pos, const osg::Vec3f& dir,
                                              const osg::Vec3f& up, bool underwater)
    {
        addWorkItem([this, pos = pos, dir = dir, up = up, underwater = underwater]
            { mImpl.setListenerPosDir(pos, dir, up, underwater); });
    }

    void AsyncSoundManager::updatePtr(const MWWorld::ConstPtr& old, const MWWorld::ConstPtr& updated)
    {
        addWorkItem([this, old = old, updated = updated] { mImpl.updatePtr(old, updated); });
    }

    void AsyncSoundManager::clear()
    {
        addWorkItem([this] { mImpl.clear(); });
    }

    template <class Function>
    void AsyncSoundManager::addWorkItem(Function&& function)
    {
        mWorkQueue.addWorkItem(makeWorkItem(std::forward<Function>(function)));
    }

    SoundRef AsyncSoundManager::playSoundAsync(MWWorld::ConstPtr ptr, const std::string& soundId,
                                               float offset, SoundPtr&& sound)
    {
        if (!sound)
            return {};

        std::list<LoadingSound>::iterator it;

        {
            const auto sounds = mLoadingSounds.lock();
            it = sounds->emplace(sounds->end());
        }

        SoundRef result = sound;

        it->mPtr = ptr;
        it->mSound = std::move(sound);
        it->mOffset = offset;
        it->mSoundId = Misc::StringUtils::lowerCase(soundId);

        addWorkItem([this, it]
        {
            if (!it->mCancelled)
                mImpl.playSoundSync(it->mPtr, it->mSoundId, it->mOffset, it->mSound);
            mLoadingSounds.lock()->erase(it);
        });

        return result;
    }
}
