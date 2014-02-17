#!/bin/bash

rostopic pub -1 -f `rospack find uibk_pick_place`/scripts/surface.msg /collision_object moveit_msgs/CollisionObject
 
 