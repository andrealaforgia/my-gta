#include "audio_device.h"

#include "log.h"

static void callback(void* user_data, Uint8* stream, int length) {
  audio_device_t* audio = user_data;
  audio_mixer_render(&audio->mixer, (float*)stream,
                     length / (int)sizeof(float));
}

bool audio_device_open(audio_device_t* audio) {
  SDL_memset(audio, 0, sizeof(*audio));
  if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
    LOG_WARN("audio", "no audio: %s", SDL_GetError());
    return false;
  }
  SDL_AudioSpec wanted, obtained;
  SDL_zero(wanted);
  wanted.freq = 48000;
  wanted.format = AUDIO_F32SYS;
  wanted.channels = 1;
  wanted.samples = 512;
  wanted.callback = callback;
  wanted.userdata = audio;
  audio->device = SDL_OpenAudioDevice(NULL, 0, &wanted, &obtained, 0);
  if (!audio->device) {
    LOG_WARN("audio", "cannot open the audio device: %s", SDL_GetError());
    return false;
  }
  audio->mixer = audio_mixer_create(obtained.freq);
  SDL_PauseAudioDevice(audio->device, 0);
  LOG_INFO("audio", "%d Hz, %d frame buffer", obtained.freq, obtained.samples);
  return true;
}

void audio_device_close(audio_device_t* audio) {
  if (audio->device) {
    SDL_PauseAudioDevice(audio->device, 1);
    SDL_CloseAudioDevice(audio->device);
    audio->device = 0;
  }
  SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

void audio_device_set_state(audio_device_t* audio, audio_state_t state) {
  if (!audio->device) return;
  if (audio->muted) state.master_volume = 0;
  SDL_LockAudioDevice(audio->device);
  audio_mixer_set_state(&audio->mixer, state);
  SDL_UnlockAudioDevice(audio->device);
}

void audio_device_trigger_impact(audio_device_t* audio, float strength) {
  if (!audio->device) return;
  SDL_LockAudioDevice(audio->device);
  audio_mixer_trigger_impact(&audio->mixer, strength);
  SDL_UnlockAudioDevice(audio->device);
}

void audio_device_toggle_mute(audio_device_t* audio) {
  audio->muted = !audio->muted;
}
