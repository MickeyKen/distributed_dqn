#!/usr/bin/env python
# -*- coding: utf-8 -*-

INITIAL_ROS_MASTER_URI = 11350

act_num = int(input('Enter actor num: '))

epsilon = float(input('Enter episilon: '))

alpha = float(input('Enter alpha: '))

output_epsilon = 0.0
OUTPUT_ROS_MASTER_URI = 0

print (" '''''' ''''''' '''''' '''''' ''''''")
for i in range(act_num):
    output_epsilon = epsilon ** (1+(i/(act_num-1)*alpha))
    print ("epsilon: ",output_epsilon)
    OUTPUT_ROS_MASTER_URI = INITIAL_ROS_MASTER_URI + i * 11
    print ("ROS_MASTER_URI: ", OUTPUT_ROS_MASTER_URI)
    print ("GAZEBO_MASTER_URI: ", OUTPUT_ROS_MASTER_URI-10)
    print (" '''''' ''''''' '''''' '''''' ''''''")
