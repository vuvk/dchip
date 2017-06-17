/*

Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

*/
module derelict.sdl.mixer;

private
{
    import derelict.util.loader;
    import derelict.util.exception;
    import derelict.util.compat;
    import derelict.sdl.sdl;
}

enum : Uint8
{
    SDL_MIXER_MAJOR_VERSION     = 1,
    SDL_MIXER_MINOR_VERSION     = 2,
    SDL_MIXER_PATCHLEVEL        = 11,
}
alias SDL_MIXER_MAJOR_VERSION MIX_MAJOR_VERSION;
alias SDL_MIXER_MINOR_VERSION MIX_MINOR_VERSION;
alias SDL_MIXER_PATCHLEVEL MIX_PATCH_LEVEL;

alias SDL_SetError Mix_SetError;
alias SDL_GetError Mix_GetError;

alias int MIX_InitFlags;
enum : int
{
    MIX_INIT_FLAC = 0x00000001,
    MIX_INIT_MOD = 0x00000002,
    MIX_INIT_MP3 = 0x00000004,
    MIX_INIT_OGG = 0x00000008,
}

struct Mix_Chunk
{
   int allocated;
   Uint8* abuf;
   Uint32 alen;
   Uint8 volume;
};

alias int Mix_Fading;
enum : int
{
   MIX_NO_FADING,
   MIX_FADING_OUT,
   MIX_FADING_IN
};

alias int Mix_MusicType;
enum : int
{
   MUS_NONE,
   MUS_CMD,
   MUS_WAV,
   MUS_MOD,
   MUS_MID,
   MUS_OGG,
   MUS_MP3,
   MUS_MP3_MAD,
   MUS_FLAC,
};

struct _Mix_Music {}
alias _Mix_Music Mix_Music;

enum
{
    MIX_CHANNELS              = 8,
    MIX_DEFAULT_FREQUENCY     = 22050,
    MIX_DEFAULT_CHANNELS      = 2,
    MIX_MAX_VOLUME            = 128,
    MIX_CHANNEL_POST          = -2,
}

version (LittleEndian) {
    enum { MIX_DEFAULT_FORMAT = AUDIO_S16LSB }
} else {
    enum { MIX_DEFAULT_FORMAT = AUDIO_S16MSB }
}

string MIX_EFFECTSMAXSPEED = "MIX_EFFECTSMAXSPEED";

extern(C)
{
    alias void function(int chan, void* stream, int len, void* udata) Mix_EffectFunc_t;
    alias void function(int chan, void* udata) Mix_EffectDone_t;
}

void SDL_MIXER_VERSION(SDL_version* X)
{
    X.major = SDL_MIXER_MAJOR_VERSION;
    X.minor = SDL_MIXER_MINOR_VERSION;
    X.patch = SDL_MIXER_PATCHLEVEL;
}
alias SDL_MIXER_VERSION MIX_VERSION;


Mix_Chunk* Mix_LoadWAV(string file)
{
    return Mix_LoadWAV_RW(SDL_RWFromFile(toCString(file), toCString("rb")), 1);
}

int Mix_PlayChannel(int channel, Mix_Chunk* chunk, int loops)
{
    return Mix_PlayChannelTimed(channel, chunk, loops, -1);
}

int Mix_FadeInChannel(int channel, Mix_Chunk* chunk, int loops, int ms)
{
    return Mix_FadeInChannelTimed(channel, chunk, loops, ms, -1);
}

extern (C)
{
    alias CSDLVERPTR function() da_Mix_Linked_Version;
    alias int function(int) da_Mix_Init;
    alias void function() da_Mix_Quit;
    alias int function (int, Uint16, int, int) da_Mix_OpenAudio;
    alias int function(int) da_Mix_AllocateChannels;
    alias int function(int*, Uint16*, int*) da_Mix_QuerySpec;
    alias Mix_Chunk* function(SDL_RWops*, int) da_Mix_LoadWAV_RW;
    alias Mix_Music* function(in char*) da_Mix_LoadMUS;
    alias Mix_Music* function(SDL_RWops*) da_Mix_LoadMUS_RW;
    alias Mix_Chunk* function(Uint8*) da_Mix_QuickLoad_WAV;
    alias Mix_Chunk* function(Uint8*, Uint32) da_Mix_QuickLoad_RAW;
    alias void function(Mix_Chunk*) da_Mix_FreeChunk;
    alias void function(Mix_Music*) da_Mix_FreeMusic;
    alias int function() da_Mix_GetNumChunkDecoders;
    alias CCPTR function(int) da_Mix_GetChunkDecoder;
    alias int function() da_Mix_GetNumMusicDecoders;
    alias CCPTR function() da_Mix_GetMusicDecoder;
    alias Mix_MusicType function(in Mix_Music*) da_Mix_GetMusicType;
    alias void function(void function(void*, Uint8*, int) da_Mix_func, void*) da_Mix_SetPostMix;
    alias void function(void function(void*, Uint8*, int) da_Mix_func, void*) da_Mix_HookMusic;
    alias void function(void function() music_finished) da_Mix_HookMusicFinished;
    alias void*  function() da_Mix_GetMusicHookData;
    alias void function(void function(int channel) channel_finished) da_Mix_ChannelFinished;
    alias int function(int, Mix_EffectFunc_t, Mix_EffectDone_t, void*) da_Mix_RegisterEffect;
    alias int function(int, Mix_EffectFunc_t) da_Mix_UnregisterEffect;
    alias int function(int) da_Mix_UnregisterAllEffects;
    alias int function(int, Uint8, Uint8) da_Mix_SetPanning;
    alias int function(int, Sint16, Uint8) da_Mix_SetPosition;
    alias int function(int, Uint8) da_Mix_SetDistance;
    // alias int function(int, Uint8) da_Mix_SetReverb;
    alias int function(int, int) da_Mix_SetReverseStereo;
    alias int function(int) da_Mix_ReserveChannels;
    alias int function(int, int) da_Mix_GroupChannel;
    alias int function(int, int, int) da_Mix_GroupChannels;
    alias int function(int) da_Mix_GroupAvailable;
    alias int function(int) da_Mix_GroupCount;
    alias int function(int) da_Mix_GroupOldest;
    alias int function(int) da_Mix_GroupNewer;
    alias int function(int, Mix_Chunk*, int, int) da_Mix_PlayChannelTimed;
    alias int function(Mix_Music*, int) da_Mix_PlayMusic;
    alias int function(Mix_Music*, int, int) da_Mix_FadeInMusic;
    alias int function(Mix_Music*, int, int, double) da_Mix_FadeInMusicPos;
    alias int function(int, Mix_Chunk*, int, int, int) da_Mix_FadeInChannelTimed;
    alias int function(int, int) da_Mix_Volume;
    alias int function(Mix_Chunk*, int) da_Mix_VolumeChunk;
    alias int function(int) da_Mix_VolumeMusic;
    alias int function(int) da_Mix_HaltChannel;
    alias int function(int) da_Mix_HaltGroup;
    alias int function() da_Mix_HaltMusic;
    alias int function(int, int) da_Mix_ExpireChannel;
    alias int function(int, int) da_Mix_FadeOutChannel;
    alias int function(int, int) da_Mix_FadeOutGroup;
    alias int function(int) da_Mix_FadeOutMusic;
    alias Mix_Fading function() da_Mix_FadingMusic;
    alias Mix_Fading function(int) da_Mix_FadingChannel;
    alias void function(int) da_Mix_Pause;
    alias void function(int) da_Mix_Resume;
    alias int function(int) da_Mix_Paused;
    alias void function() da_Mix_PauseMusic;
    alias void function() da_Mix_ResumeMusic;
    alias void function() da_Mix_RewindMusic;
    alias int function() da_Mix_PausedMusic;
    alias int function(double) da_Mix_SetMusicPosition;
    alias int function(int) da_Mix_Playing;
    alias int function() da_Mix_PlayingMusic;
    alias int function(in char*) da_Mix_SetMusicCMD;
    alias int function(int) da_Mix_SetSynchroValue;
    alias int function() da_Mix_GetSynchroValue;
    alias Mix_Chunk* function(int) da_Mix_GetChunk;
    alias void function() da_Mix_CloseAudio;
}

mixin(gsharedString!() ~
"
da_Mix_Linked_Version Mix_Linked_Version;
da_Mix_Init Mix_Init;
da_Mix_Quit Mix_Quit;
da_Mix_OpenAudio Mix_OpenAudio;
da_Mix_AllocateChannels Mix_AllocateChannels;
da_Mix_QuerySpec Mix_QuerySpec;
da_Mix_LoadWAV_RW Mix_LoadWAV_RW;
da_Mix_LoadMUS Mix_LoadMUS;
da_Mix_LoadMUS_RW Mix_LoadMUS_RW;
da_Mix_QuickLoad_WAV Mix_QuickLoad_WAV;
da_Mix_QuickLoad_RAW Mix_QuickLoad_RAW;
da_Mix_FreeChunk Mix_FreeChunk;
da_Mix_FreeMusic Mix_FreeMusic;
da_Mix_GetNumChunkDecoders Mix_GetNumChunkDecoders;
da_Mix_GetChunkDecoder Mix_GetChunkDecoder;
da_Mix_GetNumMusicDecoders Mix_GetNumMusicDecoders;
da_Mix_GetMusicDecoder Mix_GetMusicDecoder;
da_Mix_GetMusicType Mix_GetMusicType;
da_Mix_SetPostMix Mix_SetPostMix;
da_Mix_HookMusic Mix_HookMusic;
da_Mix_HookMusicFinished Mix_HookMusicFinished;
da_Mix_GetMusicHookData Mix_GetMusicHookData;
da_Mix_ChannelFinished Mix_ChannelFinished;
da_Mix_RegisterEffect Mix_RegisterEffect;
da_Mix_UnregisterEffect Mix_UnregisterEffect;
da_Mix_UnregisterAllEffects Mix_UnregisterAllEffects;
da_Mix_SetPanning Mix_SetPanning;
da_Mix_SetPosition Mix_SetPosition;
da_Mix_SetDistance Mix_SetDistance;
// da_ Mix_SetReverb;
da_Mix_SetReverseStereo Mix_SetReverseStereo;
da_Mix_ReserveChannels Mix_ReserveChannels;
da_Mix_GroupChannel Mix_GroupChannel;
da_Mix_GroupChannels Mix_GroupChannels;
da_Mix_GroupAvailable Mix_GroupAvailable;
da_Mix_GroupCount Mix_GroupCount;
da_Mix_GroupOldest Mix_GroupOldest;
da_Mix_GroupNewer Mix_GroupNewer;
da_Mix_PlayChannelTimed Mix_PlayChannelTimed;
da_Mix_PlayMusic Mix_PlayMusic;
da_Mix_FadeInMusic Mix_FadeInMusic;
da_Mix_FadeInMusicPos Mix_FadeInMusicPos;
da_Mix_FadeInChannelTimed Mix_FadeInChannelTimed;
da_Mix_Volume Mix_Volume;
da_Mix_VolumeChunk Mix_VolumeChunk;
da_Mix_VolumeMusic Mix_VolumeMusic;
da_Mix_HaltChannel Mix_HaltChannel;
da_Mix_HaltGroup Mix_HaltGroup;
da_Mix_HaltMusic Mix_HaltMusic;
da_Mix_ExpireChannel Mix_ExpireChannel;
da_Mix_FadeOutChannel Mix_FadeOutChannel;
da_Mix_FadeOutGroup Mix_FadeOutGroup;
da_Mix_FadeOutMusic Mix_FadeOutMusic;
da_Mix_FadingMusic Mix_FadingMusic;
da_Mix_FadingChannel Mix_FadingChannel;
da_Mix_Pause Mix_Pause;
da_Mix_Resume Mix_Resume;
da_Mix_Paused Mix_Paused;
da_Mix_PauseMusic Mix_PauseMusic;
da_Mix_ResumeMusic Mix_ResumeMusic;
da_Mix_RewindMusic Mix_RewindMusic;
da_Mix_PausedMusic Mix_PausedMusic;
da_Mix_SetMusicPosition Mix_SetMusicPosition;
da_Mix_Playing Mix_Playing;
da_Mix_PlayingMusic Mix_PlayingMusic;
da_Mix_SetMusicCMD Mix_SetMusicCMD;
da_Mix_SetSynchroValue Mix_SetSynchroValue;
da_Mix_GetSynchroValue Mix_GetSynchroValue;
da_Mix_GetChunk Mix_GetChunk;
da_Mix_CloseAudio Mix_CloseAudio;
");

class DerelictSDLMixerLoader : SharedLibLoader
{
public:
    this()
    {
        super(
            "SDL_mixer.dll",
            "libSDL_mixer.so, libSDL_mixer-1.2.so, libSDL_mixer-1.2.so.0",
            "../Frameworks/SDL_mixer.framework/SDL_mixer, /Library/Frameworks/SDL_mixer.framework/SDL_mixer, /System/Library/Frameworks/SDL_mixer.framework/SDL_mixer"
        );
    }

protected:
    override void loadSymbols()
    {
        if(!DerelictSDL.isLoaded)
        {
            throw new SharedLibLoadException("DerelictSDL must be loaded before attempting to load DerelictSDLMixer!");
        }

        bindFunc(cast(void**)&Mix_Linked_Version, "Mix_Linked_Version");
        bindFunc(cast(void**)&Mix_Init, "Mix_Init");
        bindFunc(cast(void**)&Mix_Quit, "Mix_Quit");
        bindFunc(cast(void**)&Mix_OpenAudio, "Mix_OpenAudio");
        bindFunc(cast(void**)&Mix_AllocateChannels, "Mix_AllocateChannels");
        bindFunc(cast(void**)&Mix_QuerySpec, "Mix_QuerySpec");
        bindFunc(cast(void**)&Mix_LoadWAV_RW, "Mix_LoadWAV_RW");
        bindFunc(cast(void**)&Mix_LoadMUS, "Mix_LoadMUS");
        bindFunc(cast(void**)&Mix_LoadMUS_RW, "Mix_LoadMUS_RW");
        bindFunc(cast(void**)&Mix_QuickLoad_WAV, "Mix_QuickLoad_WAV");
        bindFunc(cast(void**)&Mix_QuickLoad_RAW, "Mix_QuickLoad_RAW");
        bindFunc(cast(void**)&Mix_FreeChunk, "Mix_FreeChunk");
        bindFunc(cast(void**)&Mix_FreeMusic, "Mix_FreeMusic");
        bindFunc(cast(void**)&Mix_GetNumChunkDecoders, "Mix_GetNumChunkDecoders");
        bindFunc(cast(void**)&Mix_GetChunkDecoder, "Mix_GetChunkDecoder");
        bindFunc(cast(void**)&Mix_GetNumMusicDecoders, "Mix_GetNumMusicDecoders");
        bindFunc(cast(void**)&Mix_GetMusicDecoder, "Mix_GetMusicDecoder");
        bindFunc(cast(void**)&Mix_GetMusicType, "Mix_GetMusicType");
        bindFunc(cast(void**)&Mix_SetPostMix, "Mix_SetPostMix");
        bindFunc(cast(void**)&Mix_HookMusic, "Mix_HookMusic");
        bindFunc(cast(void**)&Mix_HookMusicFinished, "Mix_HookMusicFinished");
        bindFunc(cast(void**)&Mix_GetMusicHookData, "Mix_GetMusicHookData");
        bindFunc(cast(void**)&Mix_ChannelFinished, "Mix_ChannelFinished");
        bindFunc(cast(void**)&Mix_RegisterEffect, "Mix_RegisterEffect");
        bindFunc(cast(void**)&Mix_UnregisterEffect, "Mix_UnregisterEffect");
        bindFunc(cast(void**)&Mix_UnregisterAllEffects, "Mix_UnregisterAllEffects");
        bindFunc(cast(void**)&Mix_SetPanning, "Mix_SetPanning");
        bindFunc(cast(void**)&Mix_SetPosition, "Mix_SetPosition");
        bindFunc(cast(void**)&Mix_SetDistance, "Mix_SetDistance");
        // bindFunc(cast(void**)&Mix_SetReverb, "Mix_SetReverb");
        bindFunc(cast(void**)&Mix_SetReverseStereo, "Mix_SetReverseStereo");
        bindFunc(cast(void**)&Mix_ReserveChannels, "Mix_ReserveChannels");
        bindFunc(cast(void**)&Mix_GroupChannel, "Mix_GroupChannel");
        bindFunc(cast(void**)&Mix_GroupChannels, "Mix_GroupChannels");
        bindFunc(cast(void**)&Mix_GroupAvailable, "Mix_GroupAvailable");
        bindFunc(cast(void**)&Mix_GroupCount, "Mix_GroupCount");
        bindFunc(cast(void**)&Mix_GroupOldest, "Mix_GroupOldest");
        bindFunc(cast(void**)&Mix_GroupNewer, "Mix_GroupNewer");
        bindFunc(cast(void**)&Mix_PlayChannelTimed, "Mix_PlayChannelTimed");
        bindFunc(cast(void**)&Mix_PlayMusic, "Mix_PlayMusic");
        bindFunc(cast(void**)&Mix_FadeInMusic, "Mix_FadeInMusic");
        bindFunc(cast(void**)&Mix_FadeInMusicPos, "Mix_FadeInMusicPos");
        bindFunc(cast(void**)&Mix_FadeInChannelTimed, "Mix_FadeInChannelTimed");
        bindFunc(cast(void**)&Mix_Volume, "Mix_Volume");
        bindFunc(cast(void**)&Mix_VolumeChunk, "Mix_VolumeChunk");
        bindFunc(cast(void**)&Mix_VolumeMusic, "Mix_VolumeMusic");
        bindFunc(cast(void**)&Mix_HaltChannel, "Mix_HaltChannel");
        bindFunc(cast(void**)&Mix_HaltGroup, "Mix_HaltGroup");
        bindFunc(cast(void**)&Mix_HaltMusic, "Mix_HaltMusic");
        bindFunc(cast(void**)&Mix_ExpireChannel, "Mix_ExpireChannel");
        bindFunc(cast(void**)&Mix_FadeOutChannel, "Mix_FadeOutChannel");
        bindFunc(cast(void**)&Mix_FadeOutGroup, "Mix_FadeOutGroup");
        bindFunc(cast(void**)&Mix_FadeOutMusic, "Mix_FadeOutMusic");
        bindFunc(cast(void**)&Mix_FadingMusic, "Mix_FadingMusic");
        bindFunc(cast(void**)&Mix_FadingChannel, "Mix_FadingChannel");
        bindFunc(cast(void**)&Mix_Pause, "Mix_Pause");
        bindFunc(cast(void**)&Mix_Resume, "Mix_Resume");
        bindFunc(cast(void**)&Mix_Paused, "Mix_Paused");
        bindFunc(cast(void**)&Mix_PauseMusic, "Mix_PauseMusic");
        bindFunc(cast(void**)&Mix_ResumeMusic, "Mix_ResumeMusic");
        bindFunc(cast(void**)&Mix_RewindMusic, "Mix_RewindMusic");
        bindFunc(cast(void**)&Mix_PausedMusic, "Mix_PausedMusic");
        bindFunc(cast(void**)&Mix_SetMusicPosition, "Mix_SetMusicPosition");
        bindFunc(cast(void**)&Mix_Playing, "Mix_Playing");
        bindFunc(cast(void**)&Mix_PlayingMusic, "Mix_PlayingMusic");
        bindFunc(cast(void**)&Mix_SetMusicCMD, "Mix_SetMusicCMD");
        bindFunc(cast(void**)&Mix_SetSynchroValue, "Mix_SetSynchroValue");
        bindFunc(cast(void**)&Mix_GetSynchroValue, "Mix_GetSynchroValue");
        bindFunc(cast(void**)&Mix_GetChunk, "Mix_GetChunk");
        bindFunc(cast(void**)&Mix_CloseAudio, "Mix_CloseAudio");
    }
}

DerelictSDLMixerLoader DerelictSDLMixer;

static this()
{
    DerelictSDLMixer = new DerelictSDLMixerLoader();
}

static ~this()
{
    if(SharedLibLoader.isAutoUnloadEnabled())
        DerelictSDLMixer.unload();
}