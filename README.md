# controllers_brubotics
Controllers developed by the summer 2020 Brubotics interns.


## How to create a new controller in this package?
### Convention
When creating a new controller, DO NOT use the same name if this name was already used for a controller of ctu. Otherwise your new controller will overwrite the one of ctu.

when I write "controller name", I mean the name of the class. It should be written without underscores, with caps, and ending with controller, like this:
> ExampleController

And when I say "controller namespace", I mean the name of the namespace, the cpp file and the folder where it's located. It should be written in lowercase with underscores, like this:
> example_controller

When I write something with brackets in a file path, the brackets should be replaced by the value of whatever is in them, for example:
> [CONTROLLER NAMESPACE].cpp = example_controller.cpp

### Writing the code
All controllers are plugins. To begin, you should create a MRS controller plugin following the `controller.h` class. As long as you have built the `mrs_workspace` package, you will have no trouble including the class declaration on your controller via `#include`, and you will also be able to use all the mrs class includes. After the code is written, you need to add the new controller to the `address.yaml` file in the `config` folder. You can also use the config folder to create config .yaml files for your controller. If you do that, you should change the `launch/controllers_brubotics.launch` to make sure the parameters for the new controller are loaded.

#### What to change in the launch file?

In the launch file, you should add your parameter files. You should put your config file `[FILE NAME].yaml` in the folder `config/default`, and if you have different configurations for different run types and/or different uav types you should add them to `config/[RUN_TYPE]/[UAV TYPE]/[FILE NAME].yaml`. If you don't have different configurations, create the file anyway and leave it empty. Then, add the following lines to the launch file, respecting the sections defined by the comments:
```
<!-- Arg definition -->
<arg name="custom_config_[CONTROLLER NAMESPACE]" default="" />

<!-- Parameter loading -->
<rosparam ns="$(env UAV_NAME)/control_manager/[CONTROLLER NAMESPACE]" file="$(find controller_brubotics)/config/default/[FILE NAME].yaml" />
<rosparam ns="$(env UAV_NAME)/control_manager/[CONTROLLER NAMESPACE]" file="$(find trackers_brubotics)/config/$(arg RUN_TYPE)/$(arg UAV_TYPE)/[FILE NAME].yaml" />
<rosparam if="$(eval not arg('custom_config_[CONTROLLER NAMESPACE]') == '')" ns="$(env UAV_NAME)/control_manager/[CONTROLLER NAMESPACE]s" file="$(arg custom_config_[CONTROLLER NAMESPACE])" />
<param name="$(env UAV_NAME)/control_manager/[CONTROLLER NAMESPACE]/enable_profiler" type="bool" value="$(arg PROFILER)" />
<remap from="~$(env UAV_NAME)/control_manager/[CONTROLLER NAMESPACE]/profiler" to="profiler" />
```

#### What to change in the address file?

Add your controller to the `config/address.yaml` file without erasing what is already there following the model:

```
[CONTROLLER NAME]:
  address: "controllers_brubotics/[CONTROLLER NAME]"
  namespace: "[CONTROLLER NAMESPACE]"
  eland_threshold: [a number] # [m], position error triggering eland
  failsafe_threshold: [a number] # [m], position error triggering failsafe land
  odometry_innovation_threshold: [a number] # [m], position odometry innovation threshold
```

### Building the plugin

There are some things worth mentioning in the `package.xml` file, in the `CMakeLists`,  and in the `plugins.xml`. Also, an example of usage of dunamic parameters is seen in the So3Controller using the file `so3_controller_interns.cfg` in the `cfg` folder.

#### In the package.xml
Our new controllers have the same dependencies as the other mrs controller plugins, so the `depend` tag is exactly the same. The `export` tag indicates that the controller should be exported as a `mrs_uav_controller` type plugin

#### In the CMakeLists
It follows precisely the same model as the `CMakeLists` file in the ```mrs_uav_controllers```
1. In the build section, via the `add_library`, `add_dependencies` and `target_link_libraries` commands, the controllers specified are built. To build a new controller, add the following commmands to the build section:

```
add_library([CONTROLLER NAMESPACE]
  [PATH TO CONTROLLER RELATIVE TO THE PACKAGE, EXAMPLE:
  src/example_controller/example_controller.cpp]
  )

add_dependencies([CONTROLLER NAMESPACE]
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries([CONTROLLER NAMESPACE]
  ${catkin_LIBRARIES}
  )
```

2. In the install command, add our controller to the list:
```
install(TARGETS
  Se3BruboticsController
  [CONTROLLER NAMESPACE]
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
```

#### In the plugins.xml
Add your controller under what's already there, following the model:
```
<library path="lib/lib[CONTROLLER NAME]">
  <class name="controllers_brubotics/[CONTROLLER NAME]" type="mrs_uav_controllers::[CONTROLLER NAMESPACE]::[CONTROLLER NAME]" base_class_type="mrs_uav_managers::Controller">
    <description>[short description of your controller]</description>
  </class>
</library>
```

#### The so3_controller_interns.cfg
This is a .cfg file, which means it is a file used for parameter configuration that catkin automatically turns into a header file with a specified name. In this case, the file stays the same as in the mrs_uav_controllers package, and the name set for the automatically generated file changes.

### Integrating the new controller with the mrs_uav_manager

The new controller has to be added to the controller list in the `mrs_uav_manager` config file `control_manager.yaml`:

```
controllers : [
  "Se3Controller",
  "MpcController",
  "NsfController",
  "FailsafeController",
  "EmergencyController",
  "[CONTROLLER NAME]",
]
```

### Launching the new controller

The `controllers_brubotics.yaml` file takes care of the parameter loading. You shold launch it via
```
roslaunch controllers_brubotics controllers_brubotics.launch
```
**before** launching `core.launch` or `control_manager.launch` from the `mrs_uav` packages.
