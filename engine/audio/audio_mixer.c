#include "audio_mixer.h"

#include <math.h>
#include <string.h>

#define CYLINDERS 4

audio_state_t audio_state_default(void) {
  audio_state_t state = {false, 0, 0, 0, 0, 2.0f, 0.8f};
  return state;
}

static void bandpass_set(bandpass_t* filter, float frequency, float q,
                         int sample_rate) {
  float omega = 2.0f * (float)M_PI * frequency / (float)sample_rate;
  float alpha = sinf(omega) / (2.0f * q);
  float a0 = 1.0f + alpha;
  filter->frequency = frequency;
  filter->q = q;
  filter->b0 = alpha / a0;
  filter->b1 = 0;
  filter->b2 = -alpha / a0;
  filter->a1 = -2.0f * cosf(omega) / a0;
  filter->a2 = (1.0f - alpha) / a0;
}

static float bandpass_run(bandpass_t* f, float x) {
  float y =
      f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
  f->x2 = f->x1;
  f->x1 = x;
  f->y2 = f->y1;
  f->y1 = y;
  return y;
}

audio_mixer_t audio_mixer_create(int sample_rate) {
  audio_mixer_t mixer;
  memset(&mixer, 0, sizeof(mixer));
  mixer.sample_rate = sample_rate;
  mixer.state = audio_state_default();
  mixer.target = mixer.state;
  mixer.noise = 0x9e3779b9u;
  mixer.rpm_smoothed = 900;
  bandpass_set(&mixer.pipe_low, 150, 1.4f, sample_rate);
  bandpass_set(&mixer.pipe_high, 420, 2.5f, sample_rate);
  bandpass_set(&mixer.skid_filter, 1100, 1.6f, sample_rate);
  return mixer;
}

void audio_mixer_set_state(audio_mixer_t* mixer, audio_state_t state) {
  mixer->target = state;
}

void audio_mixer_trigger_impact(audio_mixer_t* mixer, float strength) {
  if (strength > mixer->impact_strength) {
    mixer->impact_strength = strength > 1 ? 1 : strength;
    mixer->impact = 1.0f;
  }
}

static float noise(audio_mixer_t* mixer) {
  mixer->noise ^= mixer->noise << 13;
  mixer->noise ^= mixer->noise >> 17;
  mixer->noise ^= mixer->noise << 5;
  return (float)(mixer->noise & 0xffff) / 32768.0f - 1.0f;
}

static float soft_clip(float x) { return tanhf(x); }

static float smooth(float value, float target, float coefficient) {
  return value + (target - value) * coefficient;
}

static float distance_gain(float distance) {
  float d = distance < 1 ? 1 : distance;
  return 1.0f / (1.0f + (d - 1.0f) * (d - 1.0f) * 0.02f);
}

void audio_mixer_render(audio_mixer_t* mixer, float* out, int frames) {
  mixer->state = mixer->target;
  const audio_state_t* s = &mixer->state;
  float dt = 1.0f / (float)mixer->sample_rate;
  float smoothing = 1.0f - expf(-dt * 12.0f);
  float gain = distance_gain(s->listener_distance) * s->master_volume;
  for (int i = 0; i < frames; i++) {
    mixer->rpm_smoothed = smooth(mixer->rpm_smoothed, s->rpm, smoothing);
    mixer->throttle_smoothed =
        smooth(mixer->throttle_smoothed, s->throttle, smoothing);
    mixer->skid_smoothed =
        smooth(mixer->skid_smoothed, s->skid, smoothing * 2.0f);
    mixer->wind_smoothed =
        smooth(mixer->wind_smoothed, s->speed_mps, smoothing);

    float engine = 0;
    if (s->engine_running) {
      /* One exhaust pulse per cylinder firing: rpm / 60 * cylinders / 2. */
      float firing_rate = mixer->rpm_smoothed / 60.0f * CYLINDERS / 2.0f;
      mixer->phase += firing_rate * dt;
      if (mixer->phase >= 1.0f) {
        mixer->phase -= 1.0f;
        mixer->pulse = 1.0f + noise(mixer) * 0.15f;
        mixer->pulse_jitter = noise(mixer) * 0.02f;
      }
      float decay = expf(-dt * (900.0f + mixer->rpm_smoothed * 0.25f));
      mixer->pulse *= decay;
      float load = 0.35f + 0.65f * mixer->throttle_smoothed;
      mixer->burst_lowpass = smooth(mixer->burst_lowpass, noise(mixer), 0.12f);
      float burst = mixer->pulse * mixer->burst_lowpass * load * 1.6f;
      /* A drone at the firing rate with its octave carries the pitch; the
       * piped bursts give the engine its texture. */
      float angle = 2.0f * (float)M_PI * mixer->phase;
      float drone = sinf(angle) * 0.7f + sinf(2.0f * angle) * 0.32f +
                    sinf(3.0f * angle) * 0.14f + sinf(4.0f * angle) * 0.06f;
      float piped = bandpass_run(&mixer->pipe_low, burst) * 1.4f +
                    bandpass_run(&mixer->pipe_high, burst) * 0.5f;
      float intake_target = noise(mixer) *
                            (0.005f + 0.03f * mixer->throttle_smoothed) *
                            (mixer->rpm_smoothed / 6000.0f);
      mixer->intake_lowpass =
          smooth(mixer->intake_lowpass, intake_target, 0.06f);
      engine = (piped + drone * (0.5f + 0.5f * load) + mixer->intake_lowpass) *
               (0.5f + 0.5f * load) * 2.5f;
      /* Two gentle poles around 1.2 kHz: the note keeps its edge, the hiss
       * does not come through. */
      float cut = 1.0f - expf(-2.0f * (float)M_PI * 1200.0f * dt);
      mixer->engine_lowpass = smooth(mixer->engine_lowpass, engine, cut);
      mixer->engine_lowpass2 =
          smooth(mixer->engine_lowpass2, mixer->engine_lowpass, cut);
      engine = mixer->engine_lowpass2 * 1.3f;
    }

    float skid = bandpass_run(&mixer->skid_filter, noise(mixer)) *
                 mixer->skid_smoothed * 1.4f;

    float impact = 0;
    if (mixer->impact > 1e-4f) {
      mixer->impact_lowpass =
          smooth(mixer->impact_lowpass, noise(mixer), 0.08f);
      impact =
          mixer->impact_lowpass * mixer->impact * mixer->impact_strength * 6.0f;
      mixer->impact *= expf(-dt * 9.0f);
    } else {
      mixer->impact_strength = 0;
    }

    float wind_level = mixer->wind_smoothed / 40.0f;
    if (wind_level > 1) wind_level = 1;
    mixer->wind_lowpass = smooth(mixer->wind_lowpass, noise(mixer), 0.015f);
    float wind = mixer->wind_lowpass * wind_level * wind_level * 0.9f;

    out[i] = soft_clip((engine * 0.55f + skid * 0.5f + impact + wind) * gain);
  }
}
