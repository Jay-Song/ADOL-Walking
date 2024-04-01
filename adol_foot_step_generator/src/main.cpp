/*
 * main.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#include "adol_foot_step_generator/message_callback.h"

int main( int argc , char **argv )
{
    ros::init( argc , argv , "adol_foot_step_generator" );

    ROS_INFO("ADOL FOOT STEP GENERATOR IS EXCUTED");

    initialize();

    ros::spin();
    return 0;
}



