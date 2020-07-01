#!/bin/bash
echo "Enter file name"
read filename
declare -a run_type_list=("simulation" "uav")
declare -a uav_type_list=("eaglemk2" "f450" "f550" "m600" "t650" "eagle" "naki")

cd ~/workspace/src/Droneswarm/controllers_brubotics/config

for rtype in ${run_type_list[@]};do
  mkdir -p $rtype
  cd $rtype
  for utype in ${uav_type_list[@]};do
    mkdir -p $utype
    cd $utype
    touch $filename
    cd ..
  done
  cd ..
done
