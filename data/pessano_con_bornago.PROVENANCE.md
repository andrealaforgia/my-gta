# data/pessano_con_bornago.osm

- Source: OpenStreetMap, https://api.openstreetmap.org/api/0.6/map, fetched in
  tiles and merged by `scripts/fetch_osm.py`
- Bounding box: minlon 9.345, minlat 45.545, maxlon 9.395, maxlat 45.580
  (Pessano con Bornago and its neighbours in the Martesana, east of Milan,
  about 3.9 km by 3.9 km)
- Downloaded: 2026-10-24 (50 667 nodes, 6 983 ways, 401 relations)
- Licence: Open Database License (ODbL) 1.0, © OpenStreetMap contributors,
  https://www.openstreetmap.org/copyright
- Not committed. Re-download and cook with:

```
make new-city CITY=pessano_con_bornago BBOX="9.345 45.545 9.395 45.580"
```

The ground is flat: the Po plain here varies by a few metres, and no elevation
model was fetched for this city.
