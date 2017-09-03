#!/bin/bash

# This script allows to run several learning machines in a row
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

if [[ "$string" == *\/* ]]
then
    echo "<folder> must be a child of current directory"
    exit -1
fi


configFolder="${folder}"
if [ ! -d $configFolder ]
then
    echo "ERROR: Failed to find '$configFolder'"
    exit -1
fi

configFile="${folder}/LearningMachine.xml"
if [ ! -f $configFile ]
then
    echo "ERROR: Failed to find '$configFile'"
    exit -1
fi

i=$first_test
while [[ i -le $2 ]]
do
    newFolder=$(printf '%s_%.3d' ${folder} $i)
    mkdir $newFolder
    if [ $? -eq 0 ]
    then
        echo "Creating folder '${newFolder}'"
    else
        echo "ERROR: failed to create folder '${newFolder}'"
        exit -1
    fi
    cp -r ${configFolder}/*.xml ${newFolder}/
    # Jumping in the folder
    cd ${newFolder}
    # Running the experiment
    ~/learning_machine > lm.out 2> lm.err
    # back to previous folder
    cd ..
    ((i++))
done
