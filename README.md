# Mestrado
Repository for files and links to master research.

## Overview
The application of this research is divided in two major components:

- Patrol-robot (Husky)

    This robot will patrol the environment to locate the major obstacles in the grid map. Futhermore, this robot uses the codes: __*grid_map.cpp*__, __*semantic.cpp*__ and __*patrol.cpp*__.

- Observer-Actor-robot (Pioneer)

    This robot will go to obstacle location, information obtained and save by the patrol-robot, and see if the obstacle remain in the position, can be removed or does not remain on the location. Futhermore, this robot uses the codes: __*mark_map_sim.cpp*__, __*object_cleaner.cpp*__ and __*object_position_reader.cpp*__.

    ### Code Explanation
    In this section the function of each code in the system will be explained.

    - __*grid_map.cpp*__

        The objective of this file is to create a grid_map from the enviroment and publish this into another topic (__/map_out__) to future use.
    
    - __*semantic.cpp*__

        The objective of this file is to receive the grid_map created and mark the objects analysed with YOLOv3 on the map and save its position into a file. The new grid map is published on topic __/map_out_s__.
    
    - __*patrol.cpp*__
    
        The objective of this file is to send a list of goals to patrol-robot. Goals are sended one by one until the patrol-robot reach the position. This process will repeat until the last goal and will restart from the first goal.

    - __*mark_map_sim.cpp*__
    
        The objective of this file is
    
    - __*object_cleaner.cpp*__
    
        The objective of this file is
    
    - __*object_position_reader.cpp*__

        The objective of this file is to read the object positions saved by patrol-robot and convert to goals for the observer-actor-robot. Like __*patrol.cpp*__, this code will send to observer-actor-robot all the goals. Futhermore, when the robot reach the determined goal, it will analyze if the object is in the position previously saved. This scenario has three possibilities of actions the observer-actor-robot will do: (1) if object saved still in place, do nothing. (2) if a new object is in the place, send a command to save the position and class of object in file. (3) if no object was seen in the position read, send a command to clear object from the map. 

## Repositories
- [DiSCo-SLAM](https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM) | Framework distribuído para SLAM multi-robôs com o intuito de utilização de Lidar 3D.
- [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) | Mundo do gazebo que simula um pequeno espaço de warehouse, junto com o mundo vem os modelos para montar mundos maiores.
- [Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) |  
- [ROBOTICS EVALUATION TOOLKITS](https://github.com/wh200720041/warehouse_simulation_toolkit) | 
- [Gazebo models and worlds collection](https://github.com/chaolmu/gazebo_models_worlds_collection) | 
- [AprilTag Imgs](https://github.com/AprilRobotics/apriltag-imgs) | Repositório com todas as famílias de tags já geradas
- [Actor Collision Plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo9/examples/plugins/actor_collisions/README.md) | Biblioteca para colisão dos atores do gazebo.
- [Quaternion <--> Euler Calculator](https://www.energid.com/resources/orientation-calculator) | Calculadora de Quaternion <--> Euler.
- [How to make animated model Gazebo](https://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot) | Tutorial de como fazer um ator animado no gazebo.
- [TEB Optimization](https://mowito-navstack.readthedocs.io/en/latest/step_5c.html) | Otimizações e soluções de problemas do TEB Local Planner 
- [A*](https://www.geeksforgeeks.org/a-search-algorithm/) | Algoritmo A*
- [Social Navigation Layer ROS](http://wiki.ros.org/social_navigation_layers) | Layer de navegação levando em consideração pessoas, implementado para ser um pacote ROS.
- [Can social_navigation_layer be used in local avoidance by DWA?](https://answers.ros.org/question/209409/can-social_navigation_layer-be-used-in-local-avoidance-by-dwa/)


## Possible References

 - H. Qiu, Z. Lin and J. Li, "Semantic Map Construction via Multi-sensor Fusion," 2021 36th Youth Academic Annual Conference of Chinese Association of Automation (YAC), 2021, pp. 495-500, doi: 10.1109/YAC53711.2021.9486598.
