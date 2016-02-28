#!/bin/bash

NB_POLICIES=10

if [ ! -f PolicyConfig.xml -o ! -f TestConfig.xml ]
then
    echo "Missing PolicyConfig.xml or TestConfig.xml"
    exit 1
fi

for (( i=1; i<=$NB_POLICIES; i++ ))
do
    # Creating folders and copying config files
    mkdir policy$i policy$i/test
    cp PolicyConfig.xml policy$i/Config.xml
    cp TestConfig.xml policy$i/test/Config.xml
    cd policy$i
    echo "rosrun csa_mdp_experiments learn_from_logs config_path:=`pwd`"
    touch learn_from_logs.csv
    # Running a test
    cd test
    echo "rosrun csa_mdp_experiments forest_controller config_path:=`pwd`"
    touch forest_controller.csv
    cd ../..
done
