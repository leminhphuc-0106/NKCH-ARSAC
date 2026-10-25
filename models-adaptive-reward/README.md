# Robotic manipulation datasets for offline compositional reinforcement learning

Offline reinforcement learning (RL) is a promising direction that allows RL agents to be pre-trained from large datasets avoiding recurrence of expensive data collection. To advance the field, it is crucial to generate large-scale datasets. Compositional RL is particularly appealing for generating such large datasets, since 1) it permits creating many tasks from few components, and 2) the task structure may enable trained agents to solve new tasks by combining relevant learned components. This submission provides four offline RL datasets for simulated robotic manipulation created using the 256 tasks from CompoSuite [Mendez et al., 2022] ([https://github.com/Lifelong-ML/CompoSuite](https://github.com/Lifelong-ML/CompoSuite)). In every task in CompoSuite, a *robot* arm is used to manipulate an *object* to achieve an *objective* all while trying to avoid an *obstacle*. There are for components for each of these four axes that can be combined arbitrarily leading to a total of 256 tasks. The component choices are

* Robot: IIWA, Jaco, Kinova3, Panda
* Object: Hollow box, box, dumbbell, plate
* Objective: Push, pick and place, put in shelf, put in trashcan
* Obstacle: None, wall between robot and object, wall between goal and object, door between goal and object

The four included datasets are collected using separate agents each trained to a different degree of performance, and each dataset consists of 256 million transitions. The degrees of performance are expert data, medium data, warmstart data and replay data:

* Expert dataset: Transitions from an expert agent that was trained to achieve 90% success on every task.
* Medium dataset: Transitions from a medium agent that was trained to achieve 30% success on every task.
* Warmstart dataset: Transitions from a Soft-actor critic agent trained for a fixed duration of one million steps.
* Medium-replay-subsampled dataset: Transitions that were stored during the training of a medium agent up to 30% success.

These datasets are intended for the combined study of compositional generalization and offline reinforcement learning.

## Collection Procedure and Decription of Data and File Structure

The datasets were collected by using several deep reinforcement learning agents trained to the various degrees of performance described above on the CompoSuite benchmark ([https://github.com/Lifelong-ML/CompoSuite](https://github.com/Lifelong-ML/CompoSuite)) which builds on top of robosuite ([https://github.com/ARISE-Initiative/robosuite](https://github.com/ARISE-Initiative/robosuite)) and uses the MuJoCo simulator ([https://github.com/deepmind/mujoco](https://github.com/deepmind/mujoco)). During reinforcement learning training, we stored the data that was collected by each agent in a separate buffer for post-processing. Then, after training, to collect the expert and medium dataset, we run the trained agents for 2000 trajectories of length 500 online in the CompoSuite benchmark and store the trajectories. These add up to a total of 1 million state-transitions tuples per dataset, totalling a full 256 million datapoints per dataset. The warmstart and medium-replay-subsampled dataset contain trajectories from the stored training buffer of the SAC agent trained for a fixed duration and the medium agent respectively. For medium-replay-subsampled data, we uniformly sample trajectories from the training buffer until we reach more than 1 million transitions. Since some of the tasks have termination conditions, some of these trajectories are trunctated and not of length 500. This sometimes results in a number of sampled transitions larger than 1 million. Therefore, after sub-sampling, we artificially truncate the last trajectory and place a timeout at the final position. This can in some rare cases lead to one incorrect trajectory if the datasets are used for finite horizon experimentation. However, this truncation is required to ensure consistent dataset sizes, easy data readability and compatibility with other standard code implementations.

The four datasets are split into four tar.gz folders each yielding a total of 12 compressed folders. Every sub-folder contains all the tasks for one of the four robot arms for that dataset. In other words, every tar.gz folder contains a total of 64 tasks using the same robot arm and four tar.gz files form a full dataset. This is done to enable people to only download a part of the dataset in case they do not need all 256 tasks. For every task, the data is separately stored in an hdf5 file allowing for the usage of arbitrary task combinations and mixing of data qualities across the four datasets. Every task is contained in a folder that is named after the CompoSuite elements it uses. In other words, every task is represented as a folder named <robot><object><obstacle>_<objective> and the data for this task is inside the corresponding folder as a data.hdf5 file. The folder structure can be summarized as follows:

├ root  
│ ├ expert-iiwa-offline-comp-data  
│ │ ├ IIWA_Box_None_Push  
│ │ │ ├ data.hdf5  
│ │ ├ ...  
│ ├ expert-jaco-offline-comp-data  
│ │ ├ Jaco_Box_None_Push  
│ │ ├ ...  

│ ├ ...  

│ ├ warmstart-panda-offline-comp-data  
│ │ ├ Panda_Box_None_Push  
│ │ ├ ...  

Following D4RL ([https://github.com/Farama-Foundation/D4RL](https://github.com/Farama-Foundation/D4RL)) style representation, every hdf5 file contains the following keys required for offline reinforcement learning training:

* observations: A (1,000,000 x 93) sized array of observations in joint robot space. This includes proprioceptive robot information, object, obstacle and goal positions, as well as a multi-hot task indicator indicating the components that were used in this observation.
* actions: A (1,000,000 x 8) sized array of actions declaring target joint angles for each of the 7 degrees of freedom of the robot arms and a grasp action.
* rewards: A (1,000,000 x 1) sized array of scalar rewards defined by the objective of the current task. For a technical definition of each of these, we refer to the original CompoSuite benchmark.
* terminals: A (1,000,000 x 1) sized array indicating terminal states (reached only in push task when object is lifted too high).
* timeouts: A (1,000,000 x 1) sized array indicating timeout states (at the end of every trajectory).
* infos: : A (1,000,000 x 1) sized array of dicts containing success information. Success is achieved when any of the states in the trajectory had a reward of 1 assigned to it.

## Sharing/Access information

As mentioned before, the data was derived using a deep reinforcement learning algorithm (Proximal Policy Optimization and Soft-actor critic) as well as CompoSuite ([https://github.com/Lifelong-ML/CompoSuite](https://github.com/Lifelong-ML/CompoSuite)) which builds on top of robosuite ([https://github.com/ARISE-Initiative/robosuite](https://github.com/ARISE-Initiative/robosuite)) and uses the MuJoCo simulator ([https://github.com/deepmind/mujoco](https://github.com/deepmind/mujoco)). The datasets can, for instance, be recreated by using trained models from [https://github.com/Lifelong-ML/CompoSuite-Data](https://github.com/Lifelong-ML/CompoSuite-Data) and our recreate_data.py python script at [https://github.com/Lifelong-ML/offline-compositional-rl-datasets](https://github.com/Lifelong-ML/offline-compositional-rl-datasets).

## Usage

The data comes in standard hdf5 format which is supported with most programming languages and can be read via standard tools. The most common language of we interest for now is expected to be python due to its widespread usage in the deep learning community. For these purposes, the files can easily be read using the h5py python package ([https://pypi.org/project/h5py/](https://pypi.org/project/h5py/)).

For convenience, we provide an implementation of an offline RL environment containing a reader for the datasets as well as a generator for online environments following the D4RL style at [https://github.com/Lifelong-ML/offline-compositional-rl-datasets](https://github.com/Lifelong-ML/offline-compositional-rl-datasets).
