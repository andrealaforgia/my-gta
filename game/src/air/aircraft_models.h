/**
 * @file aircraft_models.h
 * @brief The shapes of the big and the military aircraft: the airliner, the
 * jet fighter, the twin-rotor gunship and the missile. The wheels and legs of
 * an aeroplane are a mesh of their own, so they can be folded away. Nose
 * towards -Z, the belly on the ground at y = 0; each is split by material so
 * the caller can paint it.
 */
#ifndef GAME_SRC_AIR_AIRCRAFT_MODELS_H_
#define GAME_SRC_AIR_AIRCRAFT_MODELS_H_

#include "mesh_data.h"

/* Airliner: white skin, dark gear and engines, glass, and a coloured
 * livery. */
void build_airliner(mesh_data_t* paint, mesh_data_t* trim, mesh_data_t* glass,
                    mesh_data_t* accent, mesh_data_t* gear);
/* Jet: skin, dark parts, canopy, and the afterburner's flame. */
void build_jet(mesh_data_t* paint, mesh_data_t* trim, mesh_data_t* glass,
               mesh_data_t* flame, mesh_data_t* gear);
/* Gunship: skin, dark parts, glass, and one rotor (drawn at each hub). */
void build_gunship(mesh_data_t* paint, mesh_data_t* trim, mesh_data_t* glass,
                   mesh_data_t* rotor);
/* A missile on its way, and the flame behind it. */
void build_missile(mesh_data_t* body, mesh_data_t* flame);

#endif  // GAME_SRC_AIR_AIRCRAFT_MODELS_H_
