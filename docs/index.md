---
layout: page
title: CSE571 Project
description: cse571 project
---

## Team STAB

### Team members
- Keshav Rajasekaran 
- James Goin
- Yue Zhang

### Project description

Our plan is to learn a simple navigation system for a turtlebot. Using the [Dagger](https://www.cs.cmu.edu/~sross1/publications/Ross-AIStats11-NoRegret.pdf) learning algorithm, we hope to be able to learn a model that allows the robot to navigate a hallway and enter a door, using Kinect sensory data. The Dagger algorithm is for imitation learning in which the controls of an expert (a human, in this case) is used to train a policy for a robot.

[Link to Github repository](https://github.com/yjzhang/cse571_project)

### Goals/schedule

[Weekly Progress](weekly)

#### Midterm goals 

We hope to have implemented the Dagger learning algorithm and have it working in simulation for various scenarios.

1. Implement the Dagger algorithm in simple simulated scenarios. One simple scenario is a scenario where the goal is to navigate a circular path through a track. Another simple scenario is for one robot to pursue another robot.
2. Start working with the turtlebot, and understand how to control it and how to save data from its sensors. Finish the basic Turtlebot tutorials.
3. Save a sequence of data from the Turtlebot's laser scanners for input as the state into Dagger.
4. Create a framework that can be used to train Dagger for the Turtlebot - saves images and human control inputs, and either sends human or learned controls to the robot.

#### End goal

We have trained the robot so that it is capable of autonomously navigating through a hallway and pass through an open doorway without hitting obstacles.

1. Create a program that can send movement commands (given depth images) to the Turtlebot.
2. Train a policy using Dagger for the Turtlebot.

### Division of labor

All of us are working with the turtlebot, and we are all learning how to run controls on it.

