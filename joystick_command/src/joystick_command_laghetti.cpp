// ######################################################################################################################################################## //
// ######################################################################################################################################################## //
//                                                                                                                                                          //
//                                                            The joystick command node                                                                     //                                          
//                                                                                                                                                          //
// -------------------------------------------------------------------------------------------------------------------------------------------------------- //
//                            Author:  Francesco Ruscio       Email: francesco.ruscio@phd.unipi.it     Date:  03/08/2023                                    //
//                            Author:  Simone Tani            Email: simone.tani@phd.unipi.it          Date:  03/08/2023                                    //
//                                                                                                                                                          //
// ######################################################################################################################################################## //
// ######################################################################################################################################################## //

#include "ros/ros.h"
#include <math.h>
#include <unistd.h>
#include <tf/tf.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"
#include "marta_msgs/MotionReference.h"
#include "marta_msgs/Distance.h"
#include "joystick_command/Rel_error_joystick.h"

sensor_msgs::Joy joystick_input;
bool set_multiplier = true;
double error_yaw, error_surge_speed;
double desired_yaw;
double final_yaw;
double nominal_speed = 0.25;        // [m/s]
double saturation_yaw = 45;         // [deg]
double k_multiplier = 1.0;

ros::Time rel_error_msg_time;
ros::Duration rel_error_deltaT;

int MULTIPLIER_VALUE;
bool CONTROL_YAW, CONTROL_SURGE_SPEED;
double TOLERANCE_YAW, SAFETY_TIMEOUT;

// FUNCTIONS
void get_joystick_parameters(ros::NodeHandle node_obj);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void relative_error_callback(const joystick_command::Rel_error_joystick::ConstPtr& msg)
{   
    if(msg->error_surge_speed >= nominal_speed)
    {
        error_surge_speed   = nominal_speed;
    }
    else
    {
        error_surge_speed   = msg->error_surge_speed;
    }

    error_yaw               = msg->error_yaw;
    desired_yaw             = final_yaw + error_yaw * M_PI/180;
    rel_error_msg_time      = ros::Time::now();
}


void motion_reference_callback(const marta_msgs::MotionReference::ConstPtr& msg)
{   
    final_yaw               = msg->rpy_final.yaw;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_command_laghetti");
    ros::NodeHandle node_obj; 

    ros::Publisher pub_joystick_input    = node_obj.advertise<sensor_msgs::Joy>("/drivers/joystick", 10);
    ros::Subscriber sub_relative_error   = node_obj.subscribe("/relative_error", 1, relative_error_callback);
    ros::Subscriber sub_motion_reference = node_obj.subscribe("/refgen/joy/Reference", 1, motion_reference_callback);

    ros::Rate loop_rate(20);

    get_joystick_parameters(node_obj);

    // AXES: [SWAY, SURGE, PITCH, ROLL, -, -]
    joystick_input.axes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // BUTTONS: [MULTIPLIER MAX, MULTIPLIER MEDIUM +, MULTIPLIER MEDIUM -, MULTIPLIER MIN, YAW_SX, YAW_DX, DEPTH-, DPETH+, MULTIPLIER SELECT, -, -, -]
    joystick_input.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    //SURGE AXIS:    1.0 = 0.25 m/s | 0.38 m/s | 0.5 m/s | 0.75 m/s | 
    //SWAY AXIS:     1.0 = 0.25 m/s | 0.38 m/s | 0.5 m/s | 0.75 m/s |
    //PITCH AXIS:    1.0 = 45 deg   | 
    //ROLL AXIS:     1.0 = 5 deg/s  |

    //DEPTH BUTTON:  1 = 0.125 m/s
    //YAW BUTTON:    1 = 10 deg/s --> acceleration = 0.125 rad/s^2 | 15 deg/s | 20 deg/s| 30 deg/s

    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    while(ros::ok())
    {
        ros::spinOnce();    
        loop_rate.sleep();

        rel_error_deltaT  = ros::Time::now() - rel_error_msg_time;

        joystick_input.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        joystick_input.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        if(set_multiplier)
        {
            std::cout << "SETTING MULTIPLIER" << std::endl << std::endl;

            joystick_input.buttons[8] = 1;
            joystick_input.buttons[4 - MULTIPLIER_VALUE] = 1;

            pub_joystick_input.publish(joystick_input);
            usleep(1000000);
            pub_joystick_input.publish(joystick_input);
            set_multiplier = false;

            if(MULTIPLIER_VALUE == 2)
            {
                k_multiplier = 1.5;    
            }
            else if(MULTIPLIER_VALUE == 3)
            {
                k_multiplier = 2.0;
            }
            else if(MULTIPLIER_VALUE == 4)
            {
                k_multiplier = 3.0;
            }
            nominal_speed = nominal_speed * k_multiplier; 
        }

        if(rel_error_deltaT.toSec() >= SAFETY_TIMEOUT)
        {
            joystick_input.axes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            joystick_input.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            pub_joystick_input.publish(joystick_input);
        }
        else
        {
            if(CONTROL_SURGE_SPEED)
            {
                // SURGE SPEED COMMAND
                joystick_input.axes[1] = error_surge_speed / nominal_speed;
            }
            
            if(CONTROL_YAW)
            {
                // YAW COMMAND
                double current_error_yaw = desired_yaw - final_yaw;
                if(abs(current_error_yaw) >= TOLERANCE_YAW * M_PI/180)
                {
                    if(current_error_yaw > 0)
                    {
                        joystick_input.buttons[4] = 0;
                        joystick_input.buttons[5] = 1;
                    }
                    if(current_error_yaw < 0)
                    {
                        joystick_input.buttons[4] = 1;
                        joystick_input.buttons[5] = 0;
                    }  
                }
            }
            
            pub_joystick_input.publish(joystick_input);
        }
    }

    return 0;
}


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


void get_joystick_parameters(ros::NodeHandle node_obj)
{
    // Multiplier param
    node_obj.getParam("/multiplier", MULTIPLIER_VALUE);

    // Control params
    node_obj.getParam("/control_surge_speed", CONTROL_SURGE_SPEED);
    node_obj.getParam("/control_yaw", CONTROL_YAW);

    // Tolerance params
    node_obj.getParam("/tolerance_yaw", TOLERANCE_YAW);

    // Timeout param
    node_obj.getParam("/safety_timeout", SAFETY_TIMEOUT);
}

// #################################################################################################### //
// #################################################################################################### //
// #################################################################################################### //