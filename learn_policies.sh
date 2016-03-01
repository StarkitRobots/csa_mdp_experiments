#!/bin/bash

NB_POLICIES=10

if [ ! -f PolicyConfig.xml ]
then
    echo "Missing PolicyConfig.xml"
    exit 1
fi

for (( i=1; i<=$NB_POLICIES; i++ ))
do
    echo "Creating policy $i/$NB_POLICIES"
    # Creating folders and copying config files
    mkdir -p policy$i
    cp PolicyConfig.xml policy$i/Config.xml
    cd policy$i
    rosrun csa_mdp_experiments learn_from_logs config_path:=`pwd`
    cd ..
done
