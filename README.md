# F1/10 Occupancy Grid

---

## Design
This ROS package includes several nodes. The gridmap node should incorporate a multi-layer Occupancy grid for the F1/10 stack. The gridmap\_viz node should incorporate the visualization of the gridmap in RVIZ. The gridmap\_conversion node should handle all conversion to 2d color image etc..

### Layers
1. Environment: Should be directly from whatever map is served using map_server
2. Static Obstacles: Should be objects not from the map, but rarely moves
3. Dynamic Obstacles: Should be objects moving, humans, other cars, etc.

### Cell values

### Visualization

### Conversion to 2D image for World Models experiment

---

## ROS API
## gridmap node
### Subscribed Topics
**map(nav\_msgs/OccupancyGrid)**: This node subscribes to the map published by the map_server in order to create the environment layer of the Occupancy Grid.

**map\_metadata(nav\_msgs/MapMetaData)**: This node subscribes to the map metadata for the map information including resolution, width, height, and origin of the map.

**laserscan(sensor\_msgs/LaserScan)**: The laserscan obtained by the lidar.


### Published Topics
**env\_layer(nav\_msgs/OccupancyGrid)**

**static\_layer(nav\_msgs/OccupancyGrid)**

**dynamic\_layer(nav\_msgs/OccupancyGrid)**

### Parameters

## gridmap\_viz node
### Subscribed Topics
**env\_layer(nav\_msgs/OccupancyGrid)**:

**static\_layer(nav\_msgs/OccupancyGrid)**:

**dynamic\_layer(nav\_msgs/OccupancyGrid)**:

### Published Topics
**cost\_map(visualization_msgs/Marker)**: color representation of the costmap with the three layers

### Parameters


## gridmap\_conversion node
### Subscribed Topics


### Published Topics


### Parameters