#!/bin/bash

NB_POLICIES=10

SRC_NAME="evaluation_config.xml"
DST_NAME="mre_experiment.xml"

if [ ! -f ${SRC_NAME} ]
then
    echo "Missing ${SRC_NAME}"
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
    cp ${SRC_NAME} policy$i/test/${DST_NAME}
    cd policy$i/test
    rosrun csa_mdp_experiments mre_experiment `pwd`
    cd ../..
done
