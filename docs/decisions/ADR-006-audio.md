# ADR-006: Audio

Status: accepted, 2026-10-20

## Decision

Plain SDL2 audio (`SDL_OpenAudioDevice` with a callback) and a synthesised
engine, skid and impact, ported in approach from the car-driving game. No
SDL_mixer, no sample assets.

## Why

Synthesis driven by simulated RPM and load is what the spec asks for and it
has no licence obligations. A mixer library adds nothing when every voice is
generated. Positional attenuation is a gain per voice from listener
distance.

## Revisit

If recorded samples are wanted later, add a WAV loader; the voice interface
does not change.
