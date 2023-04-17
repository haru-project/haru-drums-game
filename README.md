# Haru drums game 

## Overview
This package contains all required software for the drums game. Contains useful script to create your own beats, all the resources needed for the game and the tree of the main tree of the game. The main idea is to create a game in which two people can play with Haru playing different beats with the drum connected to an external computer.

## Package dependencies

Below is specified the required dependencies of the drums game:
- [haru_drums_ros_driver](https://github.com/haru-project/haru_drums_ros_driver) -> This package contains the necessary drivers to connect the drums to the external PC and useful scripts  for configuring the drums to be used in the game.
- [strawberry-ros-msgs](https://github.com/haru-project/strawberry-ros-msgs) -> This package contains all the necessary messages to capture the drum's midi signals.

## Run the game 

To run the game you need to start the haru-unity simulation in case you are using the robot avatar with:
>roslaunch haru_unity unity_app_launcher.launch

and execute the command below:
>roslaunch haru_drums_game drums_game.launch

## Create your own beats for the game
You can record your own beats to use in the game with the create_beats script. To execute the script you need to execute the commando below: 
> rosrun haru_drums_game create_beats.py

Once the script is running you can start recording your own song and stop it once you have finished with CTRL+C. This script will create a .wav file with the song, a .yaml file with the color combination and a midi file with all the recorded signals that will be use to create a feedback between the original song and the one played by the child. 
