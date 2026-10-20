# Asset pipeline

Source data is immutable; everything the runtime loads is produced by a
tool from source plus settings, and can be regenerated.

## Commands

```
city-import   --input <file.osm> --output <intermediate dir> [--no-service-roads] [--elevation <lattice.txt>]
city-generate --input <intermediate dir> --config assets/city/default.cfg --output <package dir> [--name <n>]
city-validate <package dir>
```

`make city EXTRACT=data/x.osm` runs all three. Exit codes: 0 ok, 1 failure,
2 bad arguments. Nothing overwrites source data.

## Intermediate model (ADR-005)

A directory with `manifest.txt`, `roads.txt`, `buildings.txt`, `areas.txt` and,
with elevation, `terrain.txt` (`grid min_x min_z dx dz cols rows`, then one
`row` of heights per line, north first). Lines are
records; `#` lines are comments.

```
road <osm way id> <class> <width m> <width inferred 0|1> <oneway> <layer> <bridge> <tunnel> <surface|-> <name|-> <point count>
p <x> <z> <osm node id>          one per point, local metres
junction <osm node id> <x> <z> <degree>
building <osm id> <height m> <height source 0 tagged|1 levels|2 default> <levels> <category> <point count> <hole count>
p <x> <z>                        anticlockwise seen from above, ring not repeated
hole <point count>               a courtyard, clockwise, followed by its p lines
area <osm id> <water|park|parking|square> <point count>
p <x> <z>
```

Road classes are motorway, primary, secondary, tertiary, residential,
service, unclassified, pedestrian, path and waterway.

The manifest records source path, import date, licence, geographic
bounds, projection origin and the import statistics (accepted, ignored,
degenerate, malformed, relations skipped, widths inferred, heights
defaulted).

## Cooked package

`manifest.txt` plus one `chunk_<column>_<row>.txt` per grid cell.

```
# city-package 1
name, source, source_date, licence, generated, generator_version, config_hash
bounds_lat_lon, origin, projection, chunk_size
spawn <x> <y> <z> <yaw>
chunk <column> <row> <file>
```

Each chunk file holds `bounds` and a list of parts:

```
part <material> <collider mesh|none> <layer> <friction> <source id> <vertex count> <index count>
v x y z nx ny nz u v
i a b c
```

Materials are names the game binds to its procedural textures: `road`,
`junction`, `pavement`, `kerb`, `grass`, `roof`, `facade0..3`, `lamp`,
`paving` (pedestrian streets, paths, squares), `parking`, `park`, `water`,
`trunk`, `leaves`, `marking`. Facade textures hold one window bay per repeat and one
storey per repeat, so walls repeat them per `window_width` and
`storey_height`; roofs repeat a terracotta tile texture every 2.5 m. The game
merges all parts of one material in a chunk into one mesh, so draw calls
are about ten per chunk. Collider parts become static triangle meshes in
the physics world with the layer and friction given.

## Generation rules

- Each road's width is fitted to its surroundings first: the nearest
  building edge to the centre line sets the corridor, and the road and
  its pavements shrink to fit (road half width never below 1.75 m,
  pavements dropped below 0.4 m). A road running alongside, nearly
  parallel and sharing no node, splits the gap between the two centre
  lines instead: the road keeps its width and gives up the pavement on
  that side, so a dual carriageway meets at a bare median. Unconstrained
  roads keep the imported width.
- At every junction (`tools/common/junction_geometry.c`) the adjacent
  road edges are intersected to find the corner points; each road is cut
  back to where both its edges clear the corners plus a 0.2 m margin,
  and the gap is filled with the exact polygon through the corners and
  road ends. Where two arms leave at a shallow angle, as the two
  carriageways of a dual road do, the cut goes as far as the edges really
  need to clear each other, so the ribbons never overlap; on the outside
  of a sharp bend the corner is bevelled instead of mitred. A straight
  continuation needs no patch; a bend of any angle gets one.
- Roads become ribbons of their fitted width on the cut polyline,
  mitred at interior points, at `road_height` above the ground.
- Pavements of the fitted width at `kerb_height` run along both edges of
  every non-service road, with vertical kerb faces (render only), and
  pavement corner fills with kerbs join neighbouring arms around each
  junction. Pavement tops are colliders, so the car can mount them;
  kerbs never trap wheels.
- Pedestrian streets are paving ribbons with no pavements; a junction
  where only pedestrian streets meet is paved too. Paths are 2 m paving
  ribbons laid 20 mm above the roads and squares they cross and take no part in
  junction layouts. Streams are water ribbons 2 cm above the ground with
  no collider.
- Areas are flat polygons: water (2 cm, no collider), parks (2.5 cm,
  no collider, tinted grass), car parks (asphalt, 1 cm below the roads,
  collider), squares (paving, 12 mm above the roads, collider).
- Parks get trees on a jittered grid every `tree_spacing` metres, kept
  clear of the park edge: a trunk with a collider on the prop layer and a
  canopy of boxes without one. `tree_spacing = 0` turns them off.
- Junctions of three or more roads get white markings 4 mm above the
  road on each drivable arm just past its cut: a stop line across the
  entering half (the whole width on a one-way road) when the arm is at
  least 12 m long, and a zebra of 0.5 m stripes when it also has
  pavements and is at least 25 m long, so crossings do not stack where
  junctions sit close together. Markings have no collider.
- Footpaths (class path) are drawn only where they are off every street:
  sampled every 2 m, stretches on a carriageway or its pavement are left
  out, so a path ends at the pavement instead of lying across the road.
- With terrain the ground is cut into 12 m cells (heights minus 5 cm), all
  surfaces are subdivided to edges under 12 m and lifted by the ground
  height under each vertex, roads and pavements are sampled every 8 m,
  buildings stand level at the height of their centre on a 3 m skirt, and
  the spawn rises with the ground. Without terrain the ground is one quad
  per chunk and nothing moves.
- Roofs: footprints that nearly fill their bounding rectangle get a hip
  or gable roof; every other building under 22 m and between 25 and 9000
  m2 gets a ridge along the long axis of its bounding rectangle, falling
  away on both sides, with the walls rising to meet it; taller or larger
  ones are flat. Material is terracotta (70 %), slate (30 %) or the flat
  bitumen texture. Facades use one of twelve styles with their own
  palette, shutters, arches, balconies and window width.
- Porticoes: with `arcade_streets` set in the config (comma separated
  names as in the model, underscores for spaces; Pordenone's config names
  Corso_Vittorio_Emanuele_Secondo), a building edge at least 6 m long that
  runs alongside such a pedestrian street within half its width plus
  3.5 m, on a building at least 8 m deep, gets a portico: the ground
  floor (1.5 storeys) is set 3 m back behind square pillars every 4.5 m,
  the upper floors overhang on the original line with the facade texture
  restarting at a clean storey, and the walkway is a paved part of its own.
  The ends are closed unless a neighbouring arcaded edge continues.
  Such buildings use the `facade_fresco` material, a lozenge-patterned
  painted front in the manner of the Corso's frescoed houses.
- The manifest lists up to 48 `label x z TEXT` lines for the map: each
  distinct street name once, in capitals, plain ASCII, cut at any `(`, at
  the middle of its longest piece, longest streets first.
- Lamp posts stand in the middle of the pavement every `lamp_spacing`
  metres, alternating sides, with an arm and head over the road; they are
  colliders on the prop layer. `lamp_spacing = 0` turns them off.
- Buildings are extruded footprints: wall quads with outward normals and a
  texture repeat per `window_width` and `storey_height`, an ear-clipped
  roof. Courtyards are bridged into the outline before ear clipping
  (`tools/common/triangulate.c`) and get walls that face into the
  courtyard. A footprint without a courtyard that fills at least 82% of
  its minimum bounding rectangle gets a pitched roof instead (gabled when
  it is a terrace under 9 m wide and 1.6 times longer than wide, with
  gable triangles on the end walls; hipped otherwise): eaves 0.35 m
  beyond the rectangle at the wall height, a ridge along the long side
  raised by 0.6 of the half span (at most 4 m), two slopes, two hips and
  the underside of the eaves. In central Pordenone that is about six
  buildings in ten. Facade variant is a stable hash of the source id. Both are
  colliders on the building layer.
- Ground tiles cover every chunk cell at y = 0 with grass.
- The spawn is the midpoint of the longest segment of the longest
  ordinary road whose middle lies in the central part of the extract,
  facing the end of that road nearer the centre of town.
- Output is deterministic for the same model and settings; `config_hash`
  in the manifest is the settings hash.

## Validation

`city-validate` checks every part has vertices and triangles, indices in
range, finite positions, unit normals and no degenerate triangles; that
vertices lie inside their chunk bounds; that the spawn is inside a chunk;
that there are road surfaces and colliders; and that source and licence
are recorded. It prints counts and exits 1 on any issue.
