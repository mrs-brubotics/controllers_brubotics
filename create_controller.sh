#!/usr/bin/env bash
helptxt="Usage of this script:
The flag -h shows this text.
create_controller.sh <CONTROLLER NAMESPACE> <CONTROLLER NAME> [--options]
Options (optional):
  -nc or --noconfig: Does not create config files
  -c or --config: Creates config files (default)"

if [ $# -lt 2 ] || [ $# -gt 3 ]
then
  echo "$helptxt"
  exit 0
fi

pluginmodel="
<library path=\"lib/lib"$2"\">
  <class name=\"controllers_brubotics/"$2"\" type=\"mrs_uav_controllers::"$1"::"$2"\" base_class_type=\"mrs_uav_managers::Controller\">
    <description>This is the "$2"</description>
  </class>
</library>"

cmakemodel="
  # "$2"

add_library("$2"
  src/"$1"/"$1".cpp
  )
add_dependencies("$2"
  \${catkin_EXPORTED_TARGETS}
  \${\${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries("$2"
  \${catkin_LIBRARIES}
  )
  "

addressmodel="
"$2":
  address: \"controllers_brubotics/$2\"
  eland_threshold: 300
  failsafe_threshold: 300
  odometry_innovation_threshold: 300
"


trackermodel="#define VERSION \"0.0.0.0\"

#include <ros/ros.h>
#include <mrs_uav_managers/controller.h>

namespace mrs_uav_trackers
{

namespace $1
{

class $2 : public mrs_uav_managers::Controller {
public:
public:
  void initialize(const ros::NodeHandle& parent_nh, const std::string name, const std::string name_space, const mrs_uav_managers::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr& last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr& uav_state, const mrs_msgs::PositionCommand::ConstPtr& control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr& new_uav_state);

  void resetDisturbanceEstimators(void);

private:

};

  //WRITE THE FUNCTIONS HERE.

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::$1::$2, mrs_uav_managers::Controller)"

packpath=$(rospack find controllers_brubotics)
cd $packpath
cd src

mkdir -p $1
cd $1

filename=$1.cpp

if test -f $filename
then
    echo "Controller already exists. Do you wish to replace it? [y/N]"
    read answer
    if [ "$answer" = "y" ] || [ "$answer" = "Y" ]
    then
      echo "Replacing controller..."
    else
      exit 0
    fi
fi

echo "$trackermodel" > $filename
cd $packpath
echo "$pluginmodel" >> plugins.xml

linenumber=$(grep -n target_link_libraries CMakeLists.txt | cut -d : -f 1)

for number in $linenumber
do
    line=$number
done

linenumber=$(grep -n \) CMakeLists.txt | cut -d : -f 1)

for number in $linenumber
do
  newline=$number
  if [ $newline -gt $line ]
  then
    break
  fi
done

newline=$(($newline+1))

split -l $newline CMakeLists.txt
echo "$cmakemodel" >> xaa

cat x* > CMakeLists.txt
rm x*

linenumber=$(grep -n "ARCHIVE DESTINATION" CMakeLists.txt | cut -d : -f 1)
linenumber=$(($linenumber-1))

split -l $linenumber CMakeLists.txt
echo "  $2" >> xaa
cat x* > CMakeLists.txt
rm x*

cd config
echo "$addressmodel" >> address.yaml

if [ "$3" = "-nc" ] || [ "$3" = "--noconfig" ]
then
  echo "Controller created"
  exit 0
fi


run_type_list="simulation uav"
uav_type_list="eaglemk2 f450 f550 m600 t650 eagle naki"


for rtype in $run_type_list
do
  mkdir -p $rtype
  cd $rtype
  for utype in $uav_type_list
  do
    mkdir -p $utype
    cd $utype
    touch $1.yaml
    cd ..
  done
  cd ..
done

cd default
touch $1.yaml

cd ../..

cd launch

argmodel="<arg name=\"custom_config_$1\" default=\"\" />"
paramodel="
<rosparam ns=\"\$(env UAV_NAME)/control_manager/$1\" file=\"\$(find controllers_brubotics)/config/default/$1.yaml\" />
<rosparam ns=\"\$(env UAV_NAME)/control_manager/$1\" file=\"\$(find controllers_brubotics)/config/\$(arg RUN_TYPE)/\$(arg UAV_TYPE)/$1.yaml\" />
<rosparam if=\"\$(eval not arg('custom_config_$1') == '')\" ns=\"\$(env UAV_NAME)/control_manager/$1\" file=\"\$(arg custom_config_$1)\" />
<param name=\"\$(env UAV_NAME)/control_manager/$1/enable_profiler\" type=\"bool\" value=\"\$(arg PROFILER)\" />
<remap from=\"~\$(env UAV_NAME)/control_manager/$1/profiler\" to=\"profiler\" />"

linenumber=$(grep -n "<!-- Parameter loading -->" controllers_brubotics.launch | cut -d : -f 1)
linenumber=$(($linenumber-2))

split -l $linenumber controllers_brubotics.launch
echo "$argmodel" >> xaa
cat x*> controllers_brubotics.launch
rm x*

linenumber=$(grep -n "<!-- Address loading -->" controllers_brubotics.launch | cut -d : -f 1)
linenumber=$(($linenumber-2))

split -l $linenumber controllers_brubotics.launch
echo "$paramodel" >> xaa
cat x* > controllers_brubotics.launch
rm x*

echo "Controller created"
