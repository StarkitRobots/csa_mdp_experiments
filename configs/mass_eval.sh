#!/bin/bash

if [ $# -lt 2 ]
then
    echo "Usage: $0 <eval_folder> <template_folder>"
    exit -1
fi

eval_folder=$1
template_folder=$2

if [ ! -d $eval_folder ]
then
    echo "ERROR: Failed to find '$eval_folder'"
    exit -1
fi

cd $eval_folder

if [ ! -d $template_folder ]
then
    echo "ERROR: Failed to find '$template_folder' in $(pwd)"
    exit -1
fi

# Searching for all trained directories without "./" prefix
training_directories=$(find . -name "[0-9][0-9][0-9]" -printf "%P\n" | sort)

for training_dir in $training_directories
do
    # Preparing evaluation directory
    eval_dir="${template_folder}_${training_dir}"
    mkdir $eval_dir
    cp ${template_folder}/* $eval_dir
    # Updating status
    echo "Evaluating $eval_dir"
    # Jumping to evaluation directory, evaluating and jumping back
    cd $eval_dir
    sed -si "s/XXX/${training_dir}/g" LearningMachine.xml
    # WARNING: learning_machine binary should be accessible from /bin or /usr/bin
    learning_machine
    cd ..
done 
