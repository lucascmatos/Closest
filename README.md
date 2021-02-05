# espeleo_collision
-------------------------

This repositry contains the packege for detect collision and publish the closest of the robot that offers any danger. The the data acquired that is used on the package comes from the laser installed on espeleo robo, the ouster vlp-16
1. A numbered list
    1. A nested numbered list
    2. Which is numbered
2. Which is numbered
 

## Scripts included on this package:
- closest.cpp : Considering the data in 3 dimenssions coming from the laser,the script calculate points of collision danger, and return massage of if there any risk of collision and the closest point of collision.

## How to interact

First of all, if any modification is needed in the distance of collision, angle of the objects for collision classification, or number of points to filter the surface of objetcts you can do that on launch file `detect_collision.launch` located in `/closest/launch`.  

Inside this file, there are five parameters that can be modified:
        * diference_limit: Is the diference of the distance between two points calculated using only the coordenates x and y.
        * angle_limit: Is the angle limit of the surface to classified an object as representing risk of collision or not.
        * surface_points_limit: Is the number of adjacent points of an object surface that need to be classified as collision point to the algorithm return the                                    oject as an obstacle.
        * distance_yellow: Is the minimum distance that an object needs to be with respect to the robot to emit a yellow warning.
        * distance_red: Is the minimum distance that an object needs to be with respect to the robot to emit a red warning.

**Topics**

The collision in package will publish the following topics:
- `/collision_status`: Publish integer numbers of 0 to 2, where "0" represents no risk of collision (Green LED) , "1" a moderate risk of collision (yellow LED) and "2" an iminent risk of collision (red LED).
- `/closest_point`: Publish the coordenates x,y and z from the closest point that represents some risk of collision.
- `/closest_point_make` : Publish the closest point classified as a collision risk fir visualization on Rviz.

**Launch Files**

To run the part of the packege responsable for detect collision you need to run the launch file:
- `detect_collision.launch`: This launch runs the code for 3D collision points, and publish messages on the topics showed above, `/collision_status`, `/closest_point` and `/closest_point_make`. To run it correctly you must change de model in the configuration file for '3D'
