# Standard reach study
This exercise will show you how to run and re-configure a "standard" reach study, which involves a robot reaching target poses on a workpiece

## Problem statement
Evaluate the reachability of a Motoman SIA20D for inspection of a vehicle hood panel

## Tasks
### 1. Run the demo in the `reach_ros` package
- [ ] Run the "setup" launch file, which loads the URDF and starts the ancillary ROS nodes (Rviz, joint state publisher, etc.)
    ```commandline
    ros2 launch reach_ros setup.launch.py
    ```
    
- [ ] Run the reach study, loading pre-generated results 
    ```commandline
    ros2 launch reach_ros start.launch.py
    ```
    
- [ ] Left-click on the interactive markers to see the robot pose at each Cartesian target

- [ ] Right-click on the interactive markers to see the score of the robot

### 2. Run the `reach_ros` demo again using the same configuration file, but generate the results from scratch to a new destination
- [ ] With the "setup" launch file still active, re-run the "start" launch file with new arguments
    ```commandline
    ros2 launch reach_ros start.launch.py results_dir:=/tmp config_name:=study_2
    ```
    How many iterations did it take the study to converge?
    
- [ ] Open the reach study database (`/tmp/study_2/reach.db.xml`) and introspect its contents

### 3. Generate a heat map for the results of the reach study
- [ ] Use [this script](https://github.com/ros-industrial/reach/blob/1.5.2/scripts/heat_map_generator.py) to generate a heat map for the results of a reach study
    ```commandline
    python3 heat_map_generator.py <reach database file> <workpiece mesh file>
    ```
  > Note: the mesh provided to this script should be the same mesh from which the reach study target poses were generated

### 4. Relax the constraint on the orientation of the tool z-axis and re-run the reach study
In the case that our reach study only requires that the tool z-axis to align with the surface normal (e.g., for processes like sanding, painting, inspection, etc.),
we would like to relax that constraint in the reach study and see how it affects the results.
We have a few options for doing this:

1. (recommended) Use the provided [`DiscretizedMoveItIKSolver`](https://github.com/ros-industrial/reach_ros2/tree/1.3.4#discretized-moveit-ik-solver) as the REACH IK plugin.
This involves modifying the reach study configuration YAML file.
2. Use a different MoveIK solver plugin that allows orientation constraints to be set.
An example is [this modification to the trac_ik MoveIt plugin](https://github.com/marip8/trac_ik).
This change involves a modification to the MoveIt `kinematics.yaml` file, like the one [here](https://github.com/ros-industrial/reach_ros2/blob/1.3.4/demo/model/kinematics.yaml).
3. Create a custom implementation of the `TargetPoseGenerator` interface that creates multiple orientations (each rotated about the surface normal) for each target position.
This is essentially the same as option 1, but requires that we create a new class.

### 5. Change the evaluation metrics of the reach study
Maybe maximizing "raw" manipulability and distance from nearest collision are not objectives that we care about.
Instead, change the objectives such that the reach study:
- Maximizes the ratio of manipulability for position only
  - This effectively means that we want the robot to be able to translate in all directions equally effectively
- Tries to keep the joints as centered in their joint range as possible

This changes involves modifying the reach study configuration YAML file.
Check out the description of the [existing evaluator plugins](https://github.com/ros-industrial/reach_ros2/blob/1.3.4/README.md#evaluation-plugins) to figure out which plugins accomplish the objectives defined above.
Re-run the reach study, saving the results to a new file, and see how the results differ from the first reach study





