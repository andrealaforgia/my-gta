/**
 * @file audio_device.h
 * @brief The SDL audio device feeding the mixer from its callback. State
 * updates from the simulation thread are handed over under the device
 * lock.
 */
#ifndef ENGINE_AUDIO_AUDIO_DEVICE_H_
#define ENGINE_AUDIO_AUDIO_DEVICE_H_

#include <SDL.h>
#include <stdbool.h>

#include "audio_mixer.h"

typedef struct {
  SDL_AudioDeviceID device;
  audio_mixer_t mixer;
  bool muted;
} audio_device_t;

bool audio_device_open(audio_device_t* audio);
void audio_device_close(audio_device_t* audio);
void audio_device_set_state(audio_device_t* audio, audio_state_t state);
void audio_device_trigger_impact(audio_device_t* audio, float strength);
void audio_device_toggle_mute(audio_device_t* audio);

#endif  // ENGINE_AUDIO_AUDIO_DEVICE_H_
