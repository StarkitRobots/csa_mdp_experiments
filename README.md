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
(cf. README), since if some dependencies are not fulfilled, it can results in
silent errors.


# How to use the programs

This repository contains code allowing to produce two different binaries

- learning_machine
- learn_from_logs

Both takes a path as parameter and receive various parameters under the form of
an xml file. Various examples can be found in the folders `configs`.

## learning_machine

This program reads its configuration in `mre_experiment.xml`. It might be used for both,
exploring a problem space using Multi-Resolution Exploration and evaluating a policy.
Custom policies such as `Random`or `ExpertApproach` might also be used with this program.

Usage: `rosrun csa_mdp_experiments learning_machine`

Launching this program requires to have a configFile named `LearningMachine.xml` in the
current folder

## learn_from_logs

This program reads its configuration in `policy_learner.xml`, it will load an history of
samples gathered by using `mre_experiment` and analyze them with the provided parameters
and finally produce one policy per dimension

Usage: `rosrun csa_mdp_experiments learn_from_logs <path>`

## Usual structure

* THIS PART IS HIGHLY OUTDATED *

Each program will read its parameter from an xml file and produce different files during
execution such as `run_logs.csv`, `rewards_logs.csv` or `time_logs.csv`. Therefore,
it is required to prepare a folder containing only the configuration file before executing
a program. The path to this folder is provided as a parameter.

A complete task involves:
- Generating samples using an exploration policy
- Computing policies from those samples
- Evaluating those policies

The initial structure of an experiment folder is the following

```
experiment_name
  mre_exploration.xml   //Used for exploration
  policy_learner.xml    //Used for learning policies from acquired samples
  evaluation_config.xml //Used for evaluating computed policies


The final structure of the created files is similar to the following:

```
experiment_name
  mre_exploration.xml   //Used for exploration
  policy_learner.xml    //Used for learning policies from acquired samples
  evaluation_config.xml //Used for evaluating computed policies
  run_logs.csv          //The sample acquired
  prepare_logs.csv      //The samples acquired when resetting the experiment (for debug)
  time_logs.csv         //The time needed for the different steps
  details               //Data acquired during the exploration process (optional)
    ...                 
  policy_1              //The first policy
    policy_learner.xml    //Copied from root
    policy_0.data         //The forest describing the policy for dimension 0
    ...                   //The forests describing the policies for dimension k
    q_values.data         //The forest describing the computed Q value
    test                  //A folder containing the results obtained with the policy
      mre_experiment.xml    //Copied from evaluation_config.xml
      run_logs.csv          //Trajectories obtained for all the test runs
      rewards_logs.csv      //The cumulative rewards for all the runs
  policy_2              //The second policy
    ...                 
  ...                   //Other policies
```

## Launching a controller experiment

In order to run a *controller* experiment (experiment using gazebo), the simulation
needs to be started before calling `mre_experiment`.

It can be run by using the following commands:
```
roslaunch rosban_simu launch_robot.launch robot:=...
roslaunch rosban_simu ..._control.launch
```
Or as a background task without gui by using the scripts `bg_simu.sh` in `rosban_simu`:
```.../rosban_simu/launch/bg_simu.sh <robot>```

Once the simulator is started, the exploration process can be started with the
`mre_experiment` binary.

## Generating multiple policies

The script `learn_policies.sh` allows to create multiple policies from a single
set of samples. It requires the presency of a config file named
`policy_learner.xml` and create all the necessary folders.

## Evaluating multiple policies

The script `evaluate_policies.sh` allows to evaluate multiple policies at once.
It requires the presency of a config file named `evaluation_config.xml` 
and the presency of generated policies.

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

The method `ExtendedProblemFactory::registerExtraProblems` in
`src/problems/extended_problem_factory.cpp` should be updated to include
the new problem.

Finally, a new configuraton file should be built (based on existing files).

# Configuration files

By using `factory patterns` and `Serializable` objects, this project allows to
customize runs without multiplying the binaries. Since the number of parameters
tend to grow very quickly, the system allows to use *incomplete* xml files when
default values are provided in the source code.

## Problem

A problem is described by its name, and eventually by sub_properties. It allows
a brief description such as `<problem>cart_pole</problem>` if no default values
need to be overriden. It also allows more complex initialization with explicit
modification of the default parameters:
```
<problem>
  <approach>
    <max_pos>1.5</max_pos>
  </approach>
</problem>
```

## MreMachine

The configuration of a MreMachine contains some mandatory parameters:
- mode: `exploration` or `evaluation`
- update_rule: `each` or `square`
- nb_runs:
- nb_steps: the maximal number of steps per run
- problem: cf. above

Other parameters are mandatory in certain situation:
- policy: if `mode` is `evaluation`
- mre_config: if `mode` is `exploration`

And finally, some parameters are optional:
- save_details: if enabled, then save the q_value and the knownness after each policy update

## Others

Configurations of other objects should be explained later.