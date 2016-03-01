#!/bin/bash

NB_POLICIES=10

if [ ! -f TestConfig.xml ]
then
    echo "Missing TestConfig.xml"
    exit 1
fi

for (( i=1; i<=$NB_POLICIES; i++ ))
do

    if [ ! -d policy$i ]
    then
        echo "Missing folder policy$i"
        exit 1
    fi
    # Creating folder and copying config file
    mkdir -p policy$i/test
    cp TestConfig.xml policy$i/test/Config.xml
    cd policy$i/test
    rosrun csa_mdp_experiments forests_bb_evaluator config_path:=`pwd`
    cd ../..
done
