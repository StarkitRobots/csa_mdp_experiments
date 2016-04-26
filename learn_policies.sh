#!/bin/bash

NB_POLICIES=10

SRC_NAME="policy_learner.xml"
DST_NAME="policy_learner.xml"

if [ ! -f ${SRC_NAME} ]
then
    echo "Missing ${SRC_NAME}"
    exit 1
fi

for (( i=1; i<=$NB_POLICIES; i++ ))
do
    echo "Creating policy $i/$NB_POLICIES"
    # Creating folders and copying config files
    mkdir -p policy$i
    cp ${SRC_NAME} policy$i/${DST_NAME}
    cd policy$i
    rosrun csa_mdp_experiments learn_from_logs `pwd`
    cd ..
done
