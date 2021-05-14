# scaled_fjt_controller: A controller to execute follow joint trajectory action

The repository contains the implementation of a FollowJointTrajectory server as a ROS controller based on the framework [_cnr_ros_control_](https://github.com/CNR-STIIMA-IRAS/cnr_ros_control)

## TOPIC/ACTION

### Follow Joint Trajectory action server
The controller manager goal, cancel, preemption of a [Follow Joint Trajectory Action](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html)

The default action name is: /[ROBOTHW_NAME]/[CONTROLLER_NAME]/follow_joint_trajectory
You may want to remap it to the name defined in MoveIt!

### override
The controller subscribes several override topics. 

For each override topic, a std_msgs/Int64 message could be sent. The message is a number between 0 and 100 (values outside the limits are saturated in the range 0-100), which scale the velocity, the acceleration, and time of the trajectory.

The global override is given by the product of the single override (for example: 10% * 5% = 0.5%)

The default override topic names are:

- speed_ovr
- safe_ovr_1
- safe_ovr_2

You can change the names and the number of the override topics by setting the parameter _overrides_ (array of string)

### Trajectory information
The controller publishes:

- Time of the unscaled trajectory. It's the nominal time of the current point of the trajectory. Type: <std_msgs::Float64>, name: /[ROBOTHW_NAME]/[CONTROLLER_NAME]/scaled_time.

- The execution ratio (or percentage). Type <std_msgs::Float64>, name /[ROBOTHW_NAME]/[CONTROLLER_NAME]execution_ratio

- The unscaled joint target is the tuple position/velocity of the nominal trajectory in the current point. Type: <sensor_msgs::JointState>, name: /[ROBOTHW_NAME]/[CONTROLLER_NAME]/unscaled_joint_target


## CONFIGURATION
``` yaml
  microinterpolator:
    type: "cnr/control/ScaledFJTPosController"
    controlled_joints: all

    ## OPTIONAL PARAMETERS
    continuity_order: 1 # optional [default=1]. Continuity order of the trajectory. 
    # 1 velocity is continouos [you should use this one if are using the default_planner_request_adapters/AddTimeParameterization in moveit_config]
    # 2 acceleration is continouos
    # 3 jerk is continouos
    # 4 snap is continouos
    overrides: ["ovr1", "ovr2", "ovr3"] #optional (default= [speed_ovr,safe_ovr_1,safe_ovr2])
    goal_tolerance: 0.001 # optional (default=0.001). it you be a scalar or any array with a value for each joint
    path_tolerance: 0.001 # optional (default=0.001). it you be a scalar or any array with a value for each joint
```

Scaled_fjt_controller is continuosly evolving. If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/issues).

## Developer Contact
### **Authors**
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)
- Nicola Pedrocchi (<mailto::nicola.pedrocchi@stiima.cnr.it>)
- Marco Faroni (<mailto::marco.faroni@stiima.cnr.it>)

## Acknowledgements
Scaled FJT controller is developed by the Joint Research Lab  UNIBS-DIMI/CNR-STIIMA

***

![EC-H2020](Documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](Documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
