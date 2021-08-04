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


## Map Comparator (Python code)
Two python scripts: mc_func.py (cointaining the required functions) and map_comparator.py (executable to compare maps with a ground truth)

### map_comparator.py
Takes as imput the path to the ground truth .pgm file, the path to the .pgm file of map to be evaluated (or to the directory that contains the different .pgm files off all the maps to evaluate), and the path of the file where the results will be saved (in .pkl format). It prints on the screen and saves in the mentioned file the average and standard deviation of metrics specified as arguments. Those arguments can be
passed in the script itself or usinf the command line (be sure to comment and uncomment the apropiate lines in map_comparator.py file).

* IMPORTANT: 

Arguments (Note that always work with ABSOLUTE paths): 

* ground_truth: Absolute path of the ground truth .pgm file (it should be in the same directory as the .yaml file)

* map_path: File/directory absolute path of the evaluated maps. In case is only one file, it should be the absolute path to the .pgm file (it should be in the same directory as the .yaml file); if it is a directory, it should contain the .pgm and .yaml files of the maps that we want to evaluate.

* file_dest: Absolute path to the file where to store the results. That will be a dictionary in a .pkl format

* align_mode: Way to proceed when aligning the maps (intersection or union), deafult intersection.

* resolution: Resolution of the maps (should be the SAME for all maps and the ground truth)

* metrics: Metrics that we want to be computed (in the script is passed as a list, in the command line just type the name of the metrics you are interested in separated by a space). OPTIONS: 'error'(count of the cells misclasified), 'accuracy', 'precision', 'recall' and 'F1' (F1-score).

* You can find specific information of this parameters and the way they are passed through the console by using the -h option (as showed below).

### Examples of use with console

* Information about the usage:
```bash
python map_comparator.py -h
```

* General use: 
```bash
map_comparator.py [-h] [-a align_mode] [-r RESOLUTION]
                         [-m METRICS [METRICS ...]]
                         ground_truth map_path f_dest
```

* Specific Example:
```bash
map_comparator.py /home/xavi/Pictures/perfect_map_0.pgm /home/xavi/Pictures/testing /home/xavi/Pictures/test.pkl -m error accuracy F1
```
Output:
{'avg_accuracy': 0.96902150690784783, 'std_dev_error': 1957.5, 'std_dev_accuracy': 0.030978493092152115, 'std_dev_F1': 0.4484536082474227, 'avg_error': 1957.5, 'avg_F1': 0.55154639175257736}


### mc_func.py

* parameters_from_terminal(): Allow to read the parameters from the terminal and return them in a dictionary.

* align_maps(): Function to align the maps and crop or pad (depending on "align_mode" parameter). IMPORTANT: Is required that the maps and the ground truth are obtained on the same "map" frame and that the resolution is the same.

* compute_diff(): Compute different metrics (depending on "metrics" parameter). OPTIONS: 'error'(count of the cells misclasified), 'accuracy', 'precision', 'recall' and 'F1' (F1-score).

* evaluate_maps(): Wrapper for all the funcitions above mentioned. It returns a dictionary with the average and standard deviation of each metric for the set of maps that have been evaluated. 
