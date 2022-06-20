#include <iostream>
#include <cctype>

#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"

#include "Elmo_EtherCAT.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_control");

    ros::NodeHandle nh;

    ros::Publisher motor_num_current_pub = nh.advertise<std_msgs::Int32>("motor_num_current", 100);
    ros::Publisher target_Current_pub = nh.advertise<std_msgs::Int16>("target_Current", 100);

    ros::Rate loop_rate(10);
    
    std_msgs::Int32 motor_num_current_msg;
    std_msgs::Int16 target_Current_msg;

    int32_t input_motor_num_current;
    int16_t input_target_Current;

    while (ros::ok())
    {
        std::cout << "Select Motor(Motornum: (num), All:" << NUM_ELMO_DRIVER << ", quit: -1)" << std::endl;
        std::cin >> input_motor_num_current;
        if(!std::cin)
        {
            std::cout << "Wrong Input, try again." << std::endl;
            std::cin.clear();
            std::cin.ignore(INT_MAX, '\n');
            continue;
        }
        else if(input_motor_num_current == -1)
        {
            motor_num_current_msg.data = (int32_t)4;
            motor_num_current_pub.publish(motor_num_current_msg);
            target_Current_msg.data = (int16_t)0;
            target_Current_pub.publish(target_Current_msg);

            printf("Quit Program...\n");
            break;
        }
        else if(input_motor_num_current >-1 && input_motor_num_current <= NUM_ELMO_DRIVER )
        {
            printf("Published Motor Number\n");
	        motor_num_current_msg.data = input_motor_num_current;
            motor_num_current_pub.publish(motor_num_current_msg);
        }
        else
        {
            printf("Wrong Input, try again.");
            continue;
        }

        std::cout << "Input target Current(-200<I<200)" << std::endl;
        std::cin >> input_target_Current;
        if(!std::cin)
        {
            std::cout << "Wrong Input, try again." << std::endl;
            std::cin.clear();
            std::cin.ignore(INT_MAX, '\n');
            continue;
        }
        else if(input_target_Current == -1)
        {
            motor_num_current_msg.data = (int32_t)NUM_ELMO_DRIVER;
            motor_num_current_pub.publish(motor_num_current_msg);
            target_Current_msg.data = (int16_t)0;
            target_Current_pub.publish(target_Current_msg);
            printf("Quit Program...\n");
            break;
        }
        else
        {
            printf("Published Target Current\n");
            target_Current_msg.data = input_target_Current;
            target_Current_pub.publish(target_Current_msg);
        }
        ros::spinOnce();
    }

    return 0;
}
