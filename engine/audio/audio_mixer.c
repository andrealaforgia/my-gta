#include "audio_mixer.h"

#include <math.h>
#include <string.h>

#define CYLINDERS 4

audio_state_t audio_state_default(void) {
  audio_state_t state = {false, 0, 0, 0,     0, 2.0f, 0.8f, 0,
                         0,     0, 0, false, 0, 0,    0};
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
  mixer.bell_age = 99.0f;
  mixer.bird_rate = 1.0f / (48000.0f * 2.5f);
  radio_init(&mixer.radio, sample_rate);
  bandpass_set(&mixer.pipe_low, 150, 1.4f, sample_rate);
  bandpass_set(&mixer.pipe_high, 420, 2.5f, sample_rate);
  bandpass_set(&mixer.skid_filter, 1300, 1.2f, sample_rate);
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

void audio_mixer_trigger_shot(audio_mixer_t* mixer) {
  mixer->shot = 1.0f;
  mixer->shot_phase = 0;
}

void audio_mixer_trigger_bell(audio_mixer_t* mixer, int strikes) {
  if (strikes <= 0 || mixer->bell_strikes_left > 0 || mixer->bell_age < 8.0f)
    return;
  mixer->bell_strikes_left = strikes > 12 ? 12 : strikes;
  mixer->bell_timer = 0;
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
  /* A negative station is tuned in but out of earshot. */
  bool radio_audible = s->radio_station > 0;
  radio_tune(&mixer->radio,
             s->radio_station < 0 ? -s->radio_station : s->radio_station);
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
                    sinf(3.0f * angle) * 0.1f;
      float piped = bandpass_run(&mixer->pipe_low, burst) * 1.4f +
                    bandpass_run(&mixer->pipe_high, burst) * 0.25f;
      float intake_target = noise(mixer) *
                            (0.005f + 0.03f * mixer->throttle_smoothed) *
                            (mixer->rpm_smoothed / 6000.0f);
      mixer->intake_lowpass =
          smooth(mixer->intake_lowpass, intake_target, 0.06f);
      engine = (piped + drone * (0.5f + 0.5f * load) + mixer->intake_lowpass) *
               (0.5f + 0.5f * load) * 2.5f;
      /* Poles at 650 Hz: a round engine note, with nothing above it to ring
       * or whistle as the revs rise and fall. */
      float cut = 1.0f - expf(-2.0f * (float)M_PI * 650.0f * dt);
      mixer->engine_lowpass = smooth(mixer->engine_lowpass, engine, cut);
      mixer->engine_lowpass2 =
          smooth(mixer->engine_lowpass2, mixer->engine_lowpass, cut);
      mixer->engine_lowpass3 =
          smooth(mixer->engine_lowpass3, mixer->engine_lowpass2, cut);
      engine = mixer->engine_lowpass3 * 1.5f;
    }

    /* Rubber scrubbing the road: a rough, wandering scrape rather than a
     * clean note. The pitch drifts at random, the level flutters fast, and
     * a good share is band-limited noise, so it reads as tyres and not as
     * a singer. */
    mixer->skid_wobble += (180.0f + 140.0f * noise(mixer)) * dt;
    if (mixer->skid_wobble >= 1.0f) mixer->skid_wobble -= 1.0f;
    mixer->skid_drift = smooth(mixer->skid_drift, noise(mixer), 0.0008f);
    float scrape_hz = 300.0f * (1.0f + 0.8f * mixer->skid_drift);
    mixer->skid_phase += scrape_hz * dt;
    if (mixer->skid_phase >= 1.0f) mixer->skid_phase -= 1.0f;
    float scrape_angle = 2.0f * (float)M_PI * mixer->skid_phase;
    float flutter =
        0.55f + 0.45f * sinf(2.0f * (float)M_PI * mixer->skid_wobble);
    float scrape =
        (sinf(scrape_angle) + 0.6f * sinf(2.7f * scrape_angle)) * flutter;
    float skid = (scrape * 0.04f +
                  bandpass_run(&mixer->skid_filter, noise(mixer)) * 0.35f) *
                 mixer->skid_smoothed * mixer->skid_smoothed;

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

    /* The town: a low rumble, cars close by, birds by day, crickets by night.
     */
    mixer->city_smoothed =
        smooth(mixer->city_smoothed, s->city, smoothing * 0.1f);
    mixer->night_smoothed =
        smooth(mixer->night_smoothed, s->night, smoothing * 0.1f);
    mixer->traffic_smoothed =
        smooth(mixer->traffic_smoothed, s->traffic, smoothing * 0.3f);
    float hush = noise(mixer);
    mixer->city_lowpass = smooth(mixer->city_lowpass, hush, 0.01f);
    mixer->city_lowpass2 =
        smooth(mixer->city_lowpass2, mixer->city_lowpass, 0.02f);
    float town = mixer->city_lowpass2 * 1.4f * mixer->city_smoothed;
    mixer->traffic_lowpass = smooth(mixer->traffic_lowpass, hush, 0.04f);
    mixer->traffic_lowpass2 =
        smooth(mixer->traffic_lowpass2, mixer->traffic_lowpass, 0.1f);
    town += mixer->traffic_lowpass2 * 2.2f * mixer->traffic_smoothed;
    float day = (1.0f - mixer->night_smoothed) * mixer->city_smoothed;
    if (mixer->bird_envelope < 0.01f && day > 0.05f &&
        (noise(mixer) * 0.5f + 0.5f) < mixer->bird_rate * 24000.0f * day) {
      mixer->bird_envelope = 1.0f;
      mixer->bird_frequency = 2200.0f + 900.0f * (noise(mixer) * 0.5f + 0.5f);
    }
    float bird = 0;
    if (mixer->bird_envelope > 0.001f) {
      mixer->bird_phase += mixer->bird_frequency * dt;
      if (mixer->bird_phase >= 1.0f) mixer->bird_phase -= 1.0f;
      mixer->bird_frequency *= 1.0f - 1.5f * dt; /* falls as it sings */
      bird = sinf(2.0f * (float)M_PI * mixer->bird_phase) *
             mixer->bird_envelope * 0.01f * day;
      mixer->bird_envelope *= expf(-dt * 38.0f);
    }
    mixer->cricket_gate += 29.0f * dt;
    if (mixer->cricket_gate >= 1.0f) mixer->cricket_gate -= 1.0f;
    mixer->cricket_phase += (float)mixer->sample_rate / 11.0f * dt;
    if (mixer->cricket_phase >= 1.0f) mixer->cricket_phase -= 1.0f;
    float cricket = sinf(2.0f * (float)M_PI * mixer->cricket_phase) *
                    (mixer->cricket_gate < 0.55f ? 1.0f : 0.0f) * 0.007f *
                    mixer->night_smoothed * mixer->city_smoothed;
    /* Rain: a wash of high noise with a patter of drops in it. */
    mixer->rain_smoothed =
        smooth(mixer->rain_smoothed, s->rain, smoothing * 0.2f);
    float drop = noise(mixer);
    mixer->rain_lowpass = smooth(mixer->rain_lowpass, drop, 0.35f);
    float wash = (drop - mixer->rain_lowpass) * 0.5f; /* highs only */
    float patter = (noise(mixer) > 0.985f ? 1.0f : 0.0f) * noise(mixer) * 0.5f;
    float rain = (wash * 0.09f + patter * 0.05f) * mixer->rain_smoothed;
    /* The horn: two reedy notes, a major third apart, that come on in a
     * few milliseconds and cut off in a few more. */
    float horn_target = s->horn ? 1.0f : 0.0f;
    mixer->horn_level += (horn_target - mixer->horn_level) *
                         (horn_target > mixer->horn_level ? 0.004f : 0.002f) *
                         48000.0f / (float)mixer->sample_rate * 6.0f;
    float horn = 0;
    if (mixer->horn_level > 0.001f) {
      mixer->horn_phase_a += 420.0f * dt;
      mixer->horn_phase_b += 530.0f * dt;
      if (mixer->horn_phase_a >= 1.0f) mixer->horn_phase_a -= 1.0f;
      if (mixer->horn_phase_b >= 1.0f) mixer->horn_phase_b -= 1.0f;
      float a = mixer->horn_phase_a * 2.0f - 1.0f,
            b = mixer->horn_phase_b * 2.0f - 1.0f;
      /* A soft saw: harmonics fall away, so it is a horn and not a buzzer. */
      float wa =
          sinf(2.0f * (float)M_PI * mixer->horn_phase_a) * 0.7f + a * 0.2f;
      float wb =
          sinf(2.0f * (float)M_PI * mixer->horn_phase_b) * 0.7f + b * 0.2f;
      horn = (wa + wb) * 0.17f * mixer->horn_level;
    }
    /* Another driver's horn: a higher pair, quick to start and stop. */
    mixer->other_horn_level +=
        (s->other_horn - mixer->other_horn_level) * 0.01f;
    float other_horn = 0;
    if (mixer->other_horn_level > 0.002f) {
      mixer->other_horn_phase_a += 370.0f * dt;
      mixer->other_horn_phase_b += 465.0f * dt;
      if (mixer->other_horn_phase_a >= 1.0f) mixer->other_horn_phase_a -= 1.0f;
      if (mixer->other_horn_phase_b >= 1.0f) mixer->other_horn_phase_b -= 1.0f;
      float wa = sinf(2.0f * (float)M_PI * mixer->other_horn_phase_a) * 0.7f +
                 (mixer->other_horn_phase_a * 2.0f - 1.0f) * 0.2f;
      float wb = sinf(2.0f * (float)M_PI * mixer->other_horn_phase_b) * 0.7f +
                 (mixer->other_horn_phase_b * 2.0f - 1.0f) * 0.2f;
      other_horn = (wa + wb) * 0.15f * mixer->other_horn_level;
    }
    /* A siren: two tones in turn, 435 and 580 Hz, a little under half a
     * second each, with the pitch gliding between them. */
    mixer->siren_level += (s->siren - mixer->siren_level) * 0.0008f * 48000.0f /
                          (float)mixer->sample_rate;
    float siren = 0;
    if (mixer->siren_level > 0.002f) {
      mixer->siren_clock += dt;
      if (mixer->siren_clock >= 0.9f) mixer->siren_clock -= 0.9f;
      float wanted = mixer->siren_clock < 0.45f ? 435.0f : 580.0f;
      if (mixer->siren_frequency < 1.0f) mixer->siren_frequency = wanted;
      mixer->siren_frequency += (wanted - mixer->siren_frequency) * 0.002f *
                                48000.0f / (float)mixer->sample_rate;
      mixer->siren_phase += mixer->siren_frequency * dt;
      if (mixer->siren_phase >= 1.0f) mixer->siren_phase -= 1.0f;
      float saw = mixer->siren_phase * 2.0f - 1.0f;
      siren =
          (sinf(2.0f * (float)M_PI * mixer->siren_phase) * 0.8f + saw * 0.15f) *
          0.13f * mixer->siren_level * mixer->siren_level;
    }
    /* The bell: a strike every three seconds, each ringing out. */
    mixer->bell_timer -= dt;
    mixer->bell_age += dt;
    if (mixer->bell_strikes_left > 0 && mixer->bell_timer <= 0) {
      mixer->bell_strikes_left--;
      mixer->bell_timer = 3.0f;
      mixer->bell_age = 0;
    }
    float bell = 0;
    if (mixer->bell_age < 8.0f) {
      static const float ratio[4] = {0.5f, 1.0f, 2.0f, 3.0f};
      static const float amp[4] = {0.55f, 1.0f, 0.65f, 0.3f};
      static const float tau[4] = {1.1f, 0.9f, 0.6f, 0.35f};
      for (int k = 0; k < 4; k++) {
        mixer->bell_phases[k] += 220.0f * ratio[k] * dt;
        if (mixer->bell_phases[k] >= 1.0f) mixer->bell_phases[k] -= 1.0f;
        bell += sinf(2.0f * (float)M_PI * mixer->bell_phases[k]) * amp[k] *
                expf(-mixer->bell_age / tau[k]);
      }
      bell *= 0.16f;
    }
    float shot = 0;
    if (mixer->shot > 1e-4f) {
      /* A crack of noise, rounded a little, over a thump that falls in
       * pitch. */
      mixer->shot_lowpass = smooth(mixer->shot_lowpass, noise(mixer), 0.45f);
      mixer->shot_phase += (60.0f + 90.0f * mixer->shot) * dt;
      mixer->shot_phase -= floorf(mixer->shot_phase);
      shot = (mixer->shot_lowpass * 1.6f +
              sinf(2.0f * (float)M_PI * mixer->shot_phase) * 0.8f) *
             mixer->shot * mixer->shot;
      mixer->shot *= expf(-dt * 22.0f);
    }
    out[i] =
        soft_clip((engine * 0.55f + skid * 0.5f + impact + wind + town + bird +
                   cricket + bell + rain + horn + other_horn + siren +
                   radio_next(&mixer->radio) * (radio_audible ? 0.5f : 0.0f) +
                   shot * 0.8f) *
                  gain);
  }
}
