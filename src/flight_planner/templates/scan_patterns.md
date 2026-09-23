# Scan Patterns

## Lawnmower Scan Pattern
A lawnmower (boustrophedon) pattern flies parallel lanes spaced `lane_spacing_m` apart, alternating direction each lane so the drone never has to fly back across the area to start the next lane. Each drone is assigned one contiguous geographic strip of the OpsArea and only lawnmower-scans its own strip — never scatter a single drone's waypoints across the whole area.

## Voronoi Scan Pattern
A Voronoi partition splits the OpsArea into one cell per drone using seed points spread across the bounding box, then each drone lawnmower-scans only its own cell. This keeps drones spatially separated and avoids overlapping coverage between drones.
