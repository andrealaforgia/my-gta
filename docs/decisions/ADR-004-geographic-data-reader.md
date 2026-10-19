# ADR-004: Geographic data reader

Status: accepted, 2026-10-20

## Decision

Read OSM XML (`.osm`) with a small purpose-built reader in C. Not PBF.

## Why

PBF needs protobuf and zlib decoding. OSM XML for a district of a few
blocks is a few megabytes, exports directly from openstreetmap.org or the
Overpass API, and its element set (nodes, ways, relations, tags) is regular
enough that a few hundred lines of C read it. The engine never sees the
reader; it loads the intermediate format from ADR-005.

## Required outcome

Load an extract, keep ways, nodes, building polygons and tags with source
IDs, reproject to local metres (equirectangular about the region centre,
adequate for a few kilometres), report accepted, ignored and malformed
feature counts.

## Not supported in the first version

Multipolygon relations (counted and skipped with a warning), PBF.
