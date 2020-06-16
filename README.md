# controllers_brubotics
Controllers developed by the summer 2020 Brubotics interns.

## How to create a new controller in this package?

### Writing the code
All controllers are plugins. To begin, you should create a MRS controller plugin following the `controller.h` class. As long as you have built the `mrs_workspace` package, you will have no trouble including the class declaration on your controller via `#include`, and you will also be able to use all the mrs class includes.

### Building the plugin

There are some things worth mentioning in the `package.xml` file, in the `CMakeLists`, in the `plugins.xml` and in the `so3_controller_interns.cfg` in the `cfg` folder.
#### In the package.xml
Our new controllers have the same dependencies as the other mrs controller plugins, so the `depend` tag is exactly the same. The `export` tag indicates that the controller should be exported as a `mrs_uav_controller` type plugin

#### In the CMakeLists
It follows precisely the same model as the `CMakeLists` file. The only differences are:
1. In the build section, for the `add_library`, `add_dependencies` and `target_link_libraries` commands, the controller specidied is our SO3 conrtroller instead of the default mrs controllers.
2. In the install command, the plugin specified is our controllers and not the default MRS Controllers
3. The name of the package is `controllers_brubotics`
Everything else is the same.

#### In the plugins.xml
It follows the same model as in the MRS controllers package, and even the type is the same. The only difference is that the name is `controllers_brubotics/[controller name]` instead of `mrs_uav_controllers/[controller name]`

#### In the so3_controller_interns.cfg
This is a .cfg file, which means it is a file used for parameter configuration that catkin automatically turns into a header file with a specified name. In this case, the file stays the same as in the mrs_uav_controllers package, and the name set for the automatically generated file changes.

### Integrating it with the mrs_uav_manager
