#ifndef GAME_SOUND_ASYNCSOUNDMANAGER_H
#define GAME_SOUND_ASYNCSOUNDMANAGER_H

#include "soundmanagerimp.hpp"

#include <components/misc/guarded.hpp>

#include <atomic>
#include <list>

namespace MWSound
{
    class AsyncSoundManager final : public MWBase::SoundManager
    {
        public:
            AsyncSoundManager(const VFS::Manager* vfs, bool useSound);

            void processChangedSettings(const Settings::CategorySettingVector& settings) final;

            void stopMusic() final;

            void streamMusic(const std::string& fileName) final;

            bool isMusicPlaying() final;

            void playPlaylist(const std::string& playList) final;

            void playTitleMusic() final;

            void say(const MWWorld::ConstPtr& reference, const std::string& fileName) final;

            void say(const std::string& fileName) final;

            bool sayActive(const MWWorld::ConstPtr& reference) const final;

            bool sayDone(const MWWorld::ConstPtr& reference) const final;

            void stopSay(const MWWorld::ConstPtr& reference) final;

            float getSaySoundLoudness(const MWWorld::ConstPtr& reference) const final;

            StreamRef playTrack(const DecoderPtr& decoder, Type type) final;

            void stopTrack(StreamPtr stream) final;

            double getTrackTimeDelay(StreamPtr stream) final;

            SoundRef playSound(const std::string& soundId, float volume, float pitch, Type type,
                               PlayMode mode, float offset) final;

            SoundRef playSound3D(const MWWorld::ConstPtr& reference, const std::string& soundId,
                                 float volume, float pitch, Type type, PlayMode mode, float offset) final;

            SoundRef playSound3D(const osg::Vec3f& initialPos, const std::string& soundId,
                                 float volume, float pitch, Type type, PlayMode mode, float offset) final;

            void stopSound(SoundPtr sound) final;

            void stopSound3D(const MWWorld::ConstPtr& reference, const std::string& soundId) final;

            void stopSound3D(const MWWorld::ConstPtr& reference) final;

            void stopSound(const MWWorld::CellStore* cell) final;

            void fadeOutSound3D(const MWWorld::ConstPtr& reference, const std::string& soundId, float duration) final;

            bool getSoundPlaying(const MWWorld::ConstPtr& reference, const std::string& soundId) const final;

            void pauseSounds(MWSound::BlockerType blocker, int types) final;

            void resumeSounds(MWSound::BlockerType blocker) final;

            void pausePlayback() final;

            void resumePlayback() final;

            void update(float duration) final;

            void setListenerPosDir(const osg::Vec3f& pos, const osg::Vec3f& dir, const osg::Vec3f& up, bool underwater) final;

            void updatePtr(const MWWorld::ConstPtr& old, const MWWorld::ConstPtr& updated) final;

            void clear() final;

        private:
            struct LoadingVoice
            {
                MWWorld::ConstPtr mPtr;
                bool mPlayLocal;
                std::string mFileName;
                StreamPtr mStream;
                std::atomic<bool> mActive;
                std::atomic<bool> mCancelled {false};
            };

            struct LoadingMusic
            {
                std::string mFileName;
                std::atomic<bool> mCancelled {false};
            };

            struct LoadingSound
            {
                MWWorld::ConstPtr mPtr;
                std::string mSoundId;
                float mOffset;
                SoundPtr mSound;
                std::atomic<bool> mCancelled {false};
            };

            MWSound::SoundManager mImpl;
            SceneUtil::WorkQueue mWorkQueue;
            Misc::ScopeGuarded<std::list<LoadingVoice>> mLoadingVoices;
            Misc::ScopeGuarded<std::list<LoadingMusic>> mLoadingMusic;
            Misc::ScopeGuarded<std::list<LoadingSound>> mLoadingSounds;

            template <class Function>
            inline void addWorkItem(Function&& function);

            inline SoundRef playSoundAsync(MWWorld::ConstPtr ptr, const std::string& soundId,
                                           float offset, SoundPtr&& sound);
    };
}

#endif
