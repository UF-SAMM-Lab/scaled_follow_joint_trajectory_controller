# scaled_fjt_controller: A controller to execute follow joint trajectory action

The repository contains the implementation of a FollowJointTrajectory server as a ROS controller based on the framework [_cnr_ros_control_](https://github.com/CNR-STIIMA-IRAS/cnr_ros_control)

## USAGE
``` yaml
  microinterpolator:
    type: "cnr/control/ScaledFJTPosController"
    controlled_joints: all
    continuity_order: 1 # optional [default=1]. Continuity order of the trajectory. 
    # 1 velocity is continouos [you should use this one if are using the default_planner_request_adapters/AddTimeParameterization in moveit_config]
    # 2 acceleration is continouos
    # 3 jerk is continouos
    # 4 snap is continouos
```

Scaled_fjt_controller is continuosly evolving. If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/issues).

## Developer Contact
### **Authors**
- Manuel Beschi (<mailto::manuel.beschi@unibs.it>)
- Marco Faroni (<mailto::marco.faroni@unibs.it>)

## Acknowledgements
Scaled FJT controller is developed by the Joint Research Lab  UNIBS-DIMI/CNR-STIIMA

***

![EC-H2020](Documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](Documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
