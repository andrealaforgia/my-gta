# data/pordenone.osm

- Source: OpenStreetMap, https://api.openstreetmap.org/api/0.6/map
- Bounding box: minlon 12.6540, minlat 45.9545, maxlon 12.6640, maxlat 45.9615
  (central Pordenone, about 780 m by 780 m)
- Downloaded: 2026-10-20
- Licence: Open Database License (ODbL) 1.0, © OpenStreetMap contributors,
  https://www.openstreetmap.org/copyright
- Not committed. Re-download with:

```
curl -A "gta-like-prototype/0.1" -o data/pordenone.osm \
  "https://api.openstreetmap.org/api/0.6/map?bbox=12.6540,45.9545,12.6640,45.9615"
```
