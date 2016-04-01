CSA MDP EXPERIMENTS
===================

This repository contains various experiments on Continuous-State and Action
Markov Decision Processes (CSA-MDP).

# Problems
Two different type of problems are concerned by this repository:
*blackbox problems* where it is possible to sample the next state according
to any (state, action) input, and *control problems* where the only interaction
allowed to the system is to apply an action to the current state. For the second
category, we apply the actions on the effectors of a robot simulated in gazebo.

# Dependencies
This package uses catkin as build system but has no necessary depedencies
to ROS. It requires the presence of *rosban_csa_mdp* and its dependencies.

If one wish to use *control problems* (simulated in gazebo), it is necessary to
install ros entirely and to download the two following packages and their
dependencies.

- rosban_control
- rosban_simu

A specific attention should be brought to the dependencies of *rosban_simu*
(cf. README), since if some dependencies are not fulfilled, it can results with
silent errors.


# How to use the programs

This repository contains three different types of programs

- Sample Generators
  1. Random actions
  2. Based on Multiple Resolution Exploration (MRE)
- Policy Learner
  - Load samples from a file and learn a policy
- Policy Evaluation
  - Test several runs on a given problem with a specified policy

## Usual structure

Each program will create and write in different files such as `logs.csv` or
`rewards.csv`. It will also read the content of an xml file called `Config.xml`
in order to configure its parameters. Therefore, it is required to prepare a
folder containing only the `Config.xml` file in order to execute a program. The
path to this folder is provided as a parameter.

A complete task involves:
- Generating samples using an exploration policy
- Computing policies from those samples
- Evaluating those policies

The final structure of the created files is similar to the following:

```
experiment_name
  Config.xml    //The configuration used for exploration
  logs.csv      //The sample acquired
  time_logs.csv //The time needed for the different steps
  details       //Data acquired during the exploration process (folder)
    ...
  policy_1      //The first policy
    Config.xml    //The configuration of the learning process
    policy_0.data //The forest describing the policy
    q_values.data //The forest describing the compute Q value
    test          //A folder containing the results obtained with the policy
      Config.xml    //The configuration used to test the problem
      logs.csv      //Trajectories obtained for all the test runs
      rewards.csv   //The cumulative rewards for all the runs
  policy_2      //The second policy
    ...
  ...           //Other policies
```

## Launching a blackbox experiment

## Launching a controller experiment

## Learning a policy

## Evaluating policies

# Creating a new problem

...