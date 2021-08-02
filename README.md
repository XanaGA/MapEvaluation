# MapEvaluation

## Transform Broadcasters
ROS package that contains the c++ files that are necessaries for the having the exact localization of the robot.
Consist on 3 types of nodes, each of them broadcast a specific transform.

### Additional files

* bringup.launch: Launch file of the simulation

* rtabmap_simulation_xperfect.launch: Launch file to make rtabmap using the truth localization (not odometry) 

* truth_localization.launch: Launch file for the nodes needed for the truth localization

## Map saving
Once the map is created can be easily  saved by the following command: 


```bash
rosrun map_server map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] map:=/your/costmap/topic
```
Example: 
```bash
rosrun map_server map_saver --occ 60 --free 59 -f map_4cam map:=/rtabmap/grid_map
```

This will create two files:
* .pgm: Containing the information provided by the occupancy grid in image format. It is basically the map itself.

* .yaml: Cointaining information about how to read the .pgm file.

For more information go to: 
[ROS map_server package](http://wiki.ros.org/map_server)

