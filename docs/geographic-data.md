# Geographic data

## Source and licence

OpenStreetMap data, Open Database License 1.0. Attribution "© OpenStreetMap
contributors" is required wherever the generated city is shown or
distributed, and ODbL share-alike may apply to a distributed derived
database. The package manifest carries the licence line and the game logs
it at load. Review the obligations before any public release.

- https://www.openstreetmap.org/copyright
- https://opendatacommons.org/licenses/odbl/

Raw extracts live in `data/` and are not committed; each has a
`<name>.PROVENANCE.md` next to it with the URL, bounding box, date and
licence. The committed fixture `tests/fixtures/district.osm` is
hand-written, not OSM data.

## Getting an extract

The OSM API serves small bounding boxes as XML:

```
curl -A "gta-like-prototype/0.1" -o data/<name>.osm \
  "https://api.openstreetmap.org/api/0.6/map?bbox=<minlon>,<minlat>,<maxlon>,<maxlat>"
```

Keep it to a few blocks (under about a kilometre square). PBF is not
supported (ADR-004); convert with `osmium cat x.osm.pbf -o x.osm` if you
have a PBF.

## What the importer reads

- Ways tagged `highway` with a drivable class: motorway, trunk, primary,
  secondary, tertiary, residential, living_street, unclassified, road
  (and their `_link` forms). Service roads (driveways, car park aisles)
  are skipped unless `--service-roads` is given: they are short, dense
  and make junctions a mess without adding anywhere to drive.
- `highway=pedestrian` ways become the `pedestrian` class: a paved street
  the whole width across, no separate pavements. With `area=yes` they are
  squares (areas, below).
- Footways, cycleways, paths, steps and bridleways become the `path`
  class: a narrow paved strip laid over whatever it crosses, with no
  junction geometry. Footways tagged `footway=sidewalk` or
  `footway=crossing` are skipped because the generated pavements already
  stand for them.
- Open ways tagged `waterway` stream, river, canal, ditch or drain become
  the `waterway` class: a water ribbon with no collider (streams 2.5 m,
  canals 5 m, rivers 12 m unless `width` says otherwise).
- Width from `width`, else `lanes` x 3.25 m, else a configurable default
  per class, and the result is marked inferred.
- `oneway`, `layer`, `bridge`, `tunnel`, `surface`, `name` are kept as
  metadata.
- Ways tagged `building` that are closed rings with area. Height from
  `height`, else `building:levels` x 3.2 m, else a deterministic default
  between 6 and 15 m seeded by the way id. Rings are made anticlockwise.
- Areas: closed ways and multipolygon relations tagged `natural=water`,
  `waterway=riverbank`, `landuse=reservoir|basin` (water); `leisure=park`
  and friends, `landuse=grass|forest|meadow|...`, `natural=wood|scrub`
  (park); `amenity=parking` (parking); `highway=pedestrian` with
  `area=yes` or `place=square` (square).
- Multipolygon relations tagged `building` or with an area tag have their
  `outer` member ways chained end to end into closed rings; each ring is
  a building or area with the relation's tags. `inner` rings of a
  building become courtyards of the outer ring that contains them, cut
  out of the roof with their own inward-facing walls. Rings that do not
  close are counted and skipped with a warning.

## Cleaning and the road graph

Consecutive repeated nodes and coincident points are removed; ways with
fewer than two distinct points or shorter than 0.5 m are dropped as
degenerate. A way whose end stops within two metres of
another way without sharing a node is snapped onto it: to that way's
vertex if one is in reach, otherwise onto the segment, which is split
there with a fresh node. Mappers leave such gaps and they would otherwise
become roads crossing without a junction. Ways are then split at every
node shared with another accepted way, so each road segment runs
junction to junction. A junction records
its degree, the number of segment ends meeting there. Ways on different
layers only connect where they share a node, which is how OSM models it.

## Coordinates

Equirectangular projection about the centre of the extract's bounds:
x = east metres, z = south metres (north is -z), y up. Within a few
kilometres this is accurate to well under a metre. No elevation: the
ground is flat at y = 0 (bridges and tunnels are flattened too, which is a
known limit of the first slice).
