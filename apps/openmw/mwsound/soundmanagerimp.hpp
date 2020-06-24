#ifndef GAME_SOUND_SOUNDMANAGER_H
#define GAME_SOUND_SOUNDMANAGER_H

#include <memory>
#include <string>
#include <utility>
#include <deque>
#include <map>
#include <unordered_map>
#include <chrono>
#include <mutex>

#include <components/settings/settings.hpp>
#include <components/misc/objectpool.hpp>
#include <components/fallback/fallback.hpp>
#include <components/sceneutil/workqueue.hpp>
#include <components/misc/guarded.hpp>

#include "../mwbase/soundmanager.hpp"

#include "regionsoundselector.hpp"
#include "watersoundupdater.hpp"
#include "type.hpp"
#include "volumesettings.hpp"

namespace VFS
{
    class Manager;
}

namespace ESM
{
    struct Sound;
    struct Cell;
}

namespace MWSound
{
    class Sound_Output;
    struct Sound_Decoder;
    class Sound;
    class Stream;
    class Sound_Buffer;

    enum Environment {
        Env_Normal,
        Env_Underwater
    };
    // Extra play flags, not intended for caller use
    enum PlayModeEx {
        Play_2D = 0,
        Play_3D = 1<<31
    };

    using MWBase::SoundPtr;
    using StreamPtr = MWBase::SoundStreamPtr;
    using MWBase::SoundRef;
    using StreamRef = MWBase::SoundStreamRef;

    // For combining PlayMode and Type flags
    inline int operator|(PlayMode a, Type b)
    {
        return static_cast<int>(a) | static_cast<int>(b);
    }

    class SoundManager
    {
        mutable std::mutex mMutex;

        const VFS::Manager* mVFS;

        std::unique_ptr<Sound_Output> mOutput;

        // Caches available music tracks by <playlist name, (sound files) >
        std::unordered_map<std::string, std::vector<std::string>> mMusicFiles;
        std::unordered_map<std::string, std::vector<int>> mMusicToPlay; // A list with music files not yet played
        std::string mLastPlayedMusic; // The music file that was last played

        VolumeSettings mVolumeSettings;

        WaterSoundUpdater mWaterSoundUpdater;

        typedef std::unique_ptr<std::deque<Sound_Buffer> > SoundBufferList;
        // List of sound buffers, grown as needed. New enties are added to the
        // back, allowing existing Sound_Buffer references/pointers to remain
        // valid.
        SoundBufferList mSoundBuffers;
        size_t mBufferCacheMin;
        size_t mBufferCacheMax;
        size_t mBufferCacheSize;

        typedef std::unordered_map<std::string,Sound_Buffer*> NameBufferMap;
        NameBufferMap mBufferNameMap;

        // NOTE: unused buffers are stored in front-newest order.
        typedef std::deque<Sound_Buffer*> SoundList;
        SoundList mUnusedBuffers;

        Misc::ObjectPool<Sound> mSounds;

        Misc::ObjectPool<Stream> mStreams;

        typedef std::pair<SoundPtr, Sound_Buffer*> SoundBufferRefPair;
        typedef std::vector<SoundBufferRefPair> SoundBufferRefPairList;
        typedef std::map<MWWorld::ConstPtr,SoundBufferRefPairList> SoundMap;
        SoundMap mActiveSounds;

        typedef std::map<MWWorld::ConstPtr, StreamPtr> SaySoundMap;
        SaySoundMap mSaySoundsQueue;
        SaySoundMap mActiveSaySounds;

        typedef std::vector<StreamPtr> TrackList;
        TrackList mActiveTracks;

        StreamPtr mMusic;
        std::string mCurrentPlaylist;

        bool mListenerUnderwater;
        osg::Vec3f mListenerPos;
        osg::Vec3f mListenerDir;
        osg::Vec3f mListenerUp;

        int mPausedSoundTypes[BlockerType::MaxCount] = {};

        SoundRef mUnderwaterSound;
        SoundRef mNearWaterSound;

        std::string mNextMusic;
        bool mPlaybackPaused;

        RegionSoundSelector mRegionSoundSelector;

        float mTimePassed = 0;

        const ESM::Cell *mLastCell = nullptr;

        Sound_Buffer *insertSound(const std::string &soundId, const ESM::Sound *sound);

        Sound_Buffer *lookupSound(const std::string &soundId) const;
        Sound_Buffer *loadSoundSync(const std::string &soundId);

        // returns a decoder to start streaming, or nullptr if the sound was not found
        DecoderPtr loadVoice(const std::string &voicefile) const;

        SoundPtr getSoundRef();
        StreamPtr getStreamRef();

        bool playVoice(DecoderPtr decoder, bool playlocal, StreamPtr stream);

        void streamMusicFull(const std::string& filename);
        void advanceMusic(const std::string& filename);
        void startRandomTitle();

        void updateSounds(float duration);
        void updateRegionSound(float duration);
        void updateWaterSound();
        void updateMusic(float duration);

        enum class WaterSoundAction
        {
            DoNothing,
            SetVolume,
            FinishSound,
            PlaySound,
        };

        std::pair<WaterSoundAction, Sound_Buffer*> getWaterSoundAction(const WaterSoundUpdate& update,
                                                                       const ESM::Cell* cell) const;

        inline void createMusicDecoder(const std::string& fileName);

        inline void playMusicFromCreatedDecoder();

        inline void playLoadedSound(const MWWorld::ConstPtr& ptr, float offset, SoundPtr&& sound, Sound_Buffer* sfx);

        inline SoundRef playSound(const std::string& soundId, float volume, float pitch,
                                  Type type = Type::Sfx, PlayMode mode = PlayMode::Normal, float offset = 0);

        inline bool isMusicPlayingUnsafe();

        inline SoundPtr makeSoundUnsafe(float volume, float pitch, Type type, PlayMode mode);

        inline void playSoundSyncUnsafe(const MWWorld::ConstPtr& ptr, const std::string& soundId,
                                        float offset, SoundPtr&& sound);

        inline void stopMusicUnsafe();

        inline void stopSayUnsafe(const MWWorld::ConstPtr &ptr);

        inline void stopSoundUnsafe(Sound_Buffer *sfx, const MWWorld::ConstPtr &ptr);

        inline void clearUnsafe();

        inline float volumeFromType(Type type) const;

        inline void saySync(const MWWorld::ConstPtr& ptr, bool playLocal, const std::string& fileName,
                            StreamPtr&& stream, SaySoundMap& map);

        SoundManager(const SoundManager &rhs);
        SoundManager& operator=(const SoundManager &rhs);

    protected:
        DecoderPtr getDecoder() const;
        friend class OpenAL_Output;

    public:
        SoundManager(const VFS::Manager* vfs, bool useSound);
        ~SoundManager();

        void processChangedSettings(const Settings::CategorySettingVector& settings);

        void stopMusic();
        ///< Stops music if it's playing

        void streamMusic(const std::string& filename);
        ///< Play a soundifle
        /// \param filename name of a sound file in "Music/" in the data directory.

        bool isMusicPlaying();
        ///< Returns true if music is playing

        void playPlaylist(const std::string &playlist);
        ///< Start playing music from the selected folder
        /// \param name of the folder that contains the playlist

        void playTitleMusic();
        ///< Start playing title music

        bool sayActive(const MWWorld::ConstPtr &reference) const;
        ///< Is actor not speaking?

        bool sayDone(const MWWorld::ConstPtr &reference) const;
        ///< For scripting backward compatibility

        void stopSay(const MWWorld::ConstPtr &reference);
        ///< Stop an actor speaking

        float getSaySoundLoudness(const MWWorld::ConstPtr& reference) const;
        ///< Check the currently playing say sound for this actor
        /// and get an average loudness value (scale [0,1]) at the current time position.
        /// If the actor is not saying anything, returns 0.

        StreamRef playTrack(const DecoderPtr& decoder, Type type);
        ///< Play a 2D audio track, using a custom decoder

        void stopTrack(StreamPtr stream);
        ///< Stop the given audio track from playing

        double getTrackTimeDelay(StreamPtr stream);
        ///< Retives the time delay, in seconds, of the audio track (must be a sound
        /// returned by \ref playTrack). Only intended to be called by the track
        /// decoder's read method.

        void stopSound(SoundPtr sound);
        ///< Stop the given sound from playing
        /// @note no-op if \a sound is null

        void stopSound3D(const MWWorld::ConstPtr &reference, const std::string& soundId);
        ///< Stop the given object from playing the given sound,

        void stopSound3D(const MWWorld::ConstPtr &reference);
        ///< Stop the given object from playing all sounds.

        void stopSound(const MWWorld::CellStore *cell);
        ///< Stop all sounds for the given cell.

        void fadeOutSound3D(const MWWorld::ConstPtr &reference, const std::string& soundId, float duration);
        ///< Fade out given sound (that is already playing) of given object
        ///< @param reference Reference to object, whose sound is faded out
        ///< @param soundId ID of the sound to fade out.
        ///< @param duration Time until volume reaches 0.

        bool getSoundPlaying(const MWWorld::ConstPtr &reference, const std::string& soundId) const;
        ///< Is the given sound currently playing on the given object?

        void pauseSounds(MWSound::BlockerType blocker, int types);
        ///< Pauses all currently playing sounds, including music.

        void resumeSounds(MWSound::BlockerType blocker);
        ///< Resumes all previously paused sounds.

        void pausePlayback();
        void resumePlayback();

        void update(float duration);

        void setListenerPosDir(const osg::Vec3f &pos, const osg::Vec3f &dir, const osg::Vec3f &up, bool underwater);

        void updatePtr (const MWWorld::ConstPtr& old, const MWWorld::ConstPtr& updated);

        void clear();

        void playSoundSync(const MWWorld::ConstPtr& ptr, const std::string& soundId, float offset, MWBase::SoundPtr sound);

        void saySync(const MWWorld::ConstPtr& ptr, bool playLocal, const std::string& fileName, StreamPtr&& stream);

        void saySync(const std::string& fileName, StreamPtr&& stream);

        SoundPtr makeSound(float volume, float pitch, Type type, PlayMode mode);

        SoundPtr makeSound3D(const MWWorld::ConstPtr& ptr, float volume, float pitch, Type type , PlayMode mode);

        SoundPtr makeSound3D(const osg::Vec3f& pos, float volume, float pitch, Type type, PlayMode mode);

        StreamPtr makeStream(const MWWorld::ConstPtr& ptr);
    };
}

#endif
