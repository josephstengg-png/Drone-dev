# QGroundControl Plan Schema

## Top-Level Structure
A `.plan` file is JSON with keys: `fileType` ("Plan"), `groundStation`, `version`, `geoFence`, `rallyPoints`, and `mission`. Preserve `geoFence` and `plannedHomePosition` exactly as given in the baseline plan — never invent new fence points or move the home position.

## Waypoint Item Schema
Each mission item is a `SimpleItem` with `command: 16` (MAV_CMD_NAV_WAYPOINT), `frame: 3` (MAV_FRAME_GLOBAL_RELATIVE_ALT), and a 7-element `params` array: `[holdTime, acceptRadius, passRadius, yaw, latitude, longitude, altitude]`. `latitude` is `params[4]`, `longitude` is `params[5]`, `altitude` is `params[6]`. `doJumpId` is a 1-based sequence number, unique and increasing across the whole mission.

## Home Position Format
`plannedHomePosition` is `[latitude, longitude, altitude]` — latitude first. This is the opposite order from a longitude-first coordinate pair, so do not reuse a `[lon, lat]` pair here.

## GeoFence Format
`geoFence.polygons` is a list of `{"inclusion": true, "polygon": [[lat, lon], ...]}` — each point is `[latitude, longitude]`. Every waypoint you return must fall inside this polygon; the guardrail rejects the whole plan otherwise.

## Multi-Drone Tagging Convention
Because a single `.plan` file holds one mission, every mission item carries a `"comment": "drone-N"` field naming which drone (1-based) flies it. The top-level `x-multiDrone` object records `{"droneCount": <int>, "assignment": "comment field on each mission item"}` so a downstream uploader can split the mission by drone before dispatch.
