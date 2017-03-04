#!/bin/bash

# The user should add a symbolic link in home directory
binary_path="~/blackbox_learner"

# This script allows to run several black_box_learning in a row
if [ $# -lt 2 ]
then
    echo "Usage: $0 <original_folder> <last_test> <optional:first_test(default=1)>"
    exit -1
fi

folder=$1
last_test=$2
first_test=1
if [ $# -gt 2 ]
then
    first_test=$3
fi

# Forbidding situations where the folder is not a child of current directory
if [[ "$string" == *\/* ]]
then
    echo "<folder> must be a child of current directory"
    exit -1
fi

configFile="${folder}/BlackBoxLearner.xml"
if [ ! -f $configFile ]
then
    echo "ERROR: Failed to find '$configFile'"
    exit -1
fi

i=$first_test
while [[ i -le $2 ]]
do
    newFolder="${folder}$i"
    mkdir $newFolder
    if [ $? -eq 0 ]
    then
        echo "Creating folder '${newFolder}'"
    else
        echo "ERROR: failed to create folder '${newFolder}'"
        exit -1
    fi
    cp $configFile ${newFolder}/BlackBoxLearner.xml
    # Jumping in the folder
    cd ${newFolder}
    # Running the experiment
    ${binary_path} > bb.out
    # back to previous folder
    cd ..
    ((i++))
done
