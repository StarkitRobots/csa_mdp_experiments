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

In order to run a *blackbox* experiment, a folder containing an appropriate
`Config.xml` file should first be created, then we can use the following
command: `rosrun csa_mdp_experiments mre_blackbox config_path:=...`

## Launching a controller experiment

In order to run a *controller* experiment, a folder containing an appropriate
`Config.xml` file should first be created. Then the gazebo simulation need to
be started:

```
roslaunch rosban_simu launch_robot.launch robot:=...
roslaunch rosban_simu ..._control.launch
```

If graphical display is not necessary, it is also possible to run both using
the script `rosban_simu/bg_simu.sh`.

Once the simulator is started, the exploration process can be started using the
following command: `rosrun csa_mdp_experiments mre_controller config_path:=...`

## Learning a policy

In order to learn a policy, one should first acquire data and then prepare a
folder containing an appropriate `Config.xml` file. Then, the learning can be
performed as following:
`rosrun csa_mdp_experiments learn_from_logs config_path:=...`

## Evaluating a policy for a blackbox problem

In order to evaluate a policy, one should first generate it and then prepare a
folder containing an appropriate `Config.xml` file. Then, the evaluation can be
performed as following:
`rosrun csa_mdp_experiments forests_bb_evaluator config_path:=...`

## Evaluating a policy for a control problem
In order to evaluate a policy, one should first generate it and then prepare a
folder containing an appropriate `Config.xml`. It is also required to start the
gazebo simulation (cf. Launching a controller experiment)

Then, the evaluation can be performed as following:
`rosrun csa_mdp_experiments forests_controller config_path:=...`

## Generating multiple policies

The script `learn_policies.sh` allows to create multiple policies from a single
set of samples. It requires the presency of a config file named
`PolicyConfig.xml` and create all the necessary folders.

## Evaluating multiple policies

The scripts `evaluate_policies_bb.sh` and `evaluate_policies_controller.sh`
allow to evaluate multiple policies in a row. It requires the presency of a
config file named `TestConfig.xml` and the presency of generated policies.

# Examples

## Complete run on a blackbox problem

This example uses the Cart-Pole Stabilization problem.

First, create a folder and move to it:
```
mkdir /tmp/test_csa_mdp
cd /tmp/test_csa_mdp
```

Then, import the default config
```
cp ~/catkin_ws/csa_mdp_experiments/configs/cart_pole_stabilization/*Config.xml .
```

Run the experiments, learn policies and evaluate them
```
rosrun csa_mdp_experiments mre_blackbox config_path:=`pwd`
~/catkin_ws/csa_mdp_experiments/learn_policies.sh
~/catkin_ws/csa_mdp_experiments/evaluate_policies_bb.sh
```

## Complete run on a controller problem

This example uses the Cart-Pole problem (Swing-Up).

First of all, launch the robot simulation and its controller
```
roslaunch rosban_simu robot_launch.launch robot:=cart_pole &
roslaunch rosban_simu cart_pole_control.launch &
```

Then, create a folder, move to it and import the config files:
```
mkdir /tmp/test_csa_mdp
cd /tmp/test_csa_mdp
cp ~/catkin_ws/csa_mdp_experiments/configs/cart_pole/*Config.xml .
```

Run the experiments, learn policies and evaluate them
```
rosrun csa_mdp_experiments mre_controller config_path:=`pwd`
~/catkin_ws/csa_mdp_experiments/learn_policies.sh
~/catkin_ws/csa_mdp_experiments/evaluate_policies_controller.sh
```

# Creating a new problem

First of all, it is required to identify if the problem is a *blackbox* problem
or a *controller* problem. Then, a new class extending `BlackBoxProblem` or
`ControlProblem` should be written. Please refer to existing examples to see
which methods need to be implemented.

The method `ProblemFactory::build` in `src/problems/problem_factory.cpp` should
be updated to include the new problem.

Finally, a new configuraton file should be built (based on existing files).
