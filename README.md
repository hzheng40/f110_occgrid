# F1/10 Occupancy Grid
## Design
This ROS node should incorporate a multi-layer Occupancy grid for the F1/10 stack.

### Layers
1. Environment: Should be directly from whatever map is served using map_server
2. Static Obstacles: Should be objects not from the map, but rarely moves
3. Dynamic Obstacles: Should be objects moving, humans, other cars, etc.

### Subscribed Topics
**~[name]/map(nav\_msgs/OccupancyGrid)**: This node subscribes to the map published by the map_server in order to create the environment layer of the Occupancy Grid.

**~[name]/map\_metadata(nav\_msgs/MapMetaData)**: This node subscribes to the map metadata for the map information including resolution, width, height, and origin of the map.


### Published Topics
**env\_layer(nav\_msgs/OccupancyGrid)**

**static\_layer(nav\_msgs/OccupancyGrid)**

**dynamic\_layer(nav\_msgs/OccupancyGrid)**

### Parameters
