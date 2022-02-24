/**
 * @file off_mission_node.cpp
 * @brief This is an offboard control example for Kerloud Auto Car
 * More information can be referred in: https://kerloud-autocar.readthedocs.io
 *
 * @author Cloudkernel Technologies (Shenzhen) Co.,Ltd, main page: <https://cloudkernel-tech.github.io/>
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>

#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <mavconn/mavlink_dialect.h>

//variables
static std::vector<geometry_msgs::PoseStamped> waypoints;
static int current_wpindex = 0;//waypoint index starts from zero
static mavros_msgs::State current_state;
static mavros_msgs::ExtendedState current_extendedstate;
static geometry_msgs::PoseStamped current_local_pos;
static mavros_msgs::RCIn rcinput;

//All in SI units
static double nav_acc_rad_xy = 0.0f;
static double nav_acc_rad_z = 0.0f;
static double nav_acc_yaw = 0.0f;
static double current_yaw = 0.0f;

static bool _flag_last_wp_reached = false; //flag to indicate last wp is reached in position phase
static bool _flag_mission_completed = false; //flag to indicate that the mission is completed

static ros::Time _phase_entry_timestamp; //entry timestamp of a mission phase

//mission phases
enum class ROVER_MISSION_PHASE {POS_PHASE, VEL_PHASE, ATTITUDE_PHASE, ACT_CONTROL_PHASE};

static ROVER_MISSION_PHASE _mission_phase=ROVER_MISSION_PHASE::POS_PHASE;


// callbacks for subscriptions
// vehicle state
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// extended state
void extendedstate_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    current_extendedstate = *msg;
}

// vehicle local position
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = *msg;

    current_yaw = tf::getYaw(current_local_pos.pose.orientation);
}

// rc input
void rc_input_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    rcinput = *msg;
}

// init wp list from yaml file
void initTagetVector(XmlRpc::XmlRpcValue &wp_list);

// update current waypoint index
void updateWaypointIndex();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "off_mission_node");
    ros::NodeHandle nh;

    /*subscriptions, publications and services*/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 5, state_cb);

    //subscription for local position
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 5, local_pose_cb);

    ros::Subscriber rc_input_sub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in", 5, rc_input_cb);

    //publication for local position setpoint
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 5);

    //publication for local velocity setpoint
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel", 5);

    //publication for attitude setpoint (attitutde & thrust)
    ros::Publisher att_sp_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 5);

    ros::Publisher thrust_sp_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/thrust", 5);


    //publication for direct actuator outputs
    ros::Publisher act_controls_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 5);

    //service for arm/disarm
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    //service for main mode setting
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //service for command send
    ros::ServiceClient command_long_client = nh.serviceClient<mavros_msgs::CommandLong>
            ("mavros/cmd/command");

    /* param loading*/
    ros::NodeHandle private_nh("~");

    // load yaml files
    XmlRpc::XmlRpcValue wp_list;
    private_nh.getParam("waypoints",wp_list);
    initTagetVector(wp_list);
    ROS_INFO("waypoint yaml loaded!");

    // update parameters
    int simulation_flag = 0;

    private_nh.getParam("simulation_flag", simulation_flag);
    private_nh.getParam("nav_acc_rad_xy", nav_acc_rad_xy);
    private_nh.getParam("nav_acc_yaw_deg", nav_acc_yaw);

    nav_acc_yaw = nav_acc_yaw/180.0f*M_PI;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    /*service commands*/
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    //forward/backward driving command
    mavros_msgs::CommandLong set_forward_driving_cmd;
    set_forward_driving_cmd.request.command = (uint16_t)mavlink::common::MAV_CMD::SET_ROVER_FORWARD_REVERSE_DRIVING;
    set_forward_driving_cmd.request.confirmation = 0;
    set_forward_driving_cmd.request.param1 = 1.0f;

    mavros_msgs::CommandLong set_backward_driving_cmd;
    set_backward_driving_cmd.request.command = (uint16_t)mavlink::common::MAV_CMD::SET_ROVER_FORWARD_REVERSE_DRIVING;
    set_backward_driving_cmd.request.confirmation = 0;
    set_backward_driving_cmd.request.param1 = 0.0f;

    ros::Time last_request = ros::Time::now();


    while(ros::ok()){

        if (current_wpindex == 0)
        {
            if (simulation_flag == 1)
            {
                //set offboard mode, then arm the vehicle
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent){
                        ROS_INFO("Offboard mode enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
            }

        }


        if (!_flag_mission_completed){

        switch (_mission_phase){

        /* publish local position setpoint for rover maneuvering*/
            case ROVER_MISSION_PHASE::POS_PHASE:{

                    ROS_INFO_ONCE("Starting position guidance phase");

                    geometry_msgs::PoseStamped pose; //pose to be passed to fcu

                    if (current_wpindex == 0){
                        waypoints.at(0).pose.position.x += current_local_pos.pose.position.x; //set with relative position here
                        waypoints.at(0).pose.position.y += current_local_pos.pose.position.y;
                        waypoints.at(0).pose.position.z += current_local_pos.pose.position.z;

                        pose = waypoints.at(0);
                    }
                    else
                        pose = waypoints.at(current_wpindex);

                    local_pos_pub.publish(pose);
                    updateWaypointIndex();

                break;
            }

         /* publish local velocity setpoint for rover maneuvering*/
        case ROVER_MISSION_PHASE::VEL_PHASE:{

                ROS_INFO_ONCE("Starting velocity guidance phase");

                //The locl velocity setpoint is defined in the ENU frame, and will be converted to body frame in the autopilot for maneuvering
                geometry_msgs::Twist  vel_cmd;
                vel_cmd.linear.x = 0.25f;
                vel_cmd.linear.y = 0.25f;
                vel_cmd.linear.z = 0.0f;

                local_vel_pub.publish(vel_cmd);

                //phase transition after a certain time
                if ( ros::Time::now() - _phase_entry_timestamp > ros::Duration(6.0)){
                    _mission_phase = ROVER_MISSION_PHASE::ATTITUDE_PHASE;
                    _phase_entry_timestamp = ros::Time::now();
                }

                break;

        }

         /* publish attitude setpoint for rover maneuvering*/
        case ROVER_MISSION_PHASE::ATTITUDE_PHASE:{
                ROS_INFO_ONCE("Starting attitude maneuvering phase");

                //we need to construct both attitude and thrust setpoints
                //we construct desired attitude from yaw setpoint
                geometry_msgs::PoseStamped pose;

                tf::Quaternion q = tf::createQuaternionFromYaw(M_PI/2.0f); //yaw unit: radius

                tf::quaternionTFToMsg(q, pose.pose.orientation);

                att_sp_pub.publish(pose);

                mavros_msgs::Thrust thrust_sp;
                thrust_sp.thrust = 0.3f;
                thrust_sp_pub.publish(thrust_sp);

                //phase transition after a certain time
                if ( ros::Time::now() - _phase_entry_timestamp > ros::Duration(6.0)){
                    _mission_phase = ROVER_MISSION_PHASE::ACT_CONTROL_PHASE;
                    _phase_entry_timestamp = ros::Time::now();
                }

            break;
        }

         /* publish direct actuator control for rover maneuvering*/
        case ROVER_MISSION_PHASE::ACT_CONTROL_PHASE:{
                ROS_INFO_ONCE("Starting direct actuator control phase");

                mavros_msgs::ActuatorControl act_control;

                act_control.group_mix = 0;
                act_control.flag_rover_mode = 1;//enable direct actuator control in rover
                act_control.controls[mavros_msgs::ActuatorControl::ROVER_YAW_CHANNEL_CONTROL_INDEX] = 0.4f;
                act_control.controls[mavros_msgs::ActuatorControl::ROVER_THROTTLE_CHANNEL_CONTROL_INDEX] = 0.3f;

                act_controls_pub.publish(act_control);

                //phase transition after a certain time
                if ( ros::Time::now() - _phase_entry_timestamp > ros::Duration(6.0)){
                    _flag_mission_completed = true;
                    ROS_INFO("Mission completed");
                }

            break;
        }


      default:

            ROS_INFO("Unsupported mission phases");
            break;


        }

        }






        /*disarm when the mission is completed*/
        if (_flag_mission_completed){

            if( current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){

                if( arming_client.call(disarm_cmd) && arm_cmd.response.success){

                    ROS_INFO("Vehicle disarmed");
                }
                last_request = ros::Time::now();

            }

        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



void initTagetVector(XmlRpc::XmlRpcValue &wp_list)
{
    waypoints.clear();

    geometry_msgs::PoseStamped tempPose;
    for (size_t i = 0; i < wp_list.size(); ++i)
    {
        tempPose.header.seq = i;
        XmlRpc::XmlRpcValue data_list(wp_list[i]);

        // get position
        tempPose.pose.position.x = data_list[0];
        tempPose.pose.position.y = data_list[1];
        tempPose.pose.position.z = data_list[2];

        // get orientation
        tf::Quaternion q = tf::createQuaternionFromYaw(data_list[3]);

        tf::quaternionTFToMsg(q, tempPose.pose.orientation);

        waypoints.push_back(tempPose);

    }

    ROS_INFO("Loaded waypoint size is %d ", (int)waypoints.size());
}


void updateWaypointIndex()
{
    float dx = current_local_pos.pose.position.x - waypoints.at(current_wpindex).pose.position.x;
    float dy = current_local_pos.pose.position.y - waypoints.at(current_wpindex).pose.position.y;
    //float yaw_sp = tf::getYaw(waypoints.at(current_wpindex).pose.orientation);

    float dist_xy_sq = dx*dx+dy*dy;

    bool is_position_reached_flag = false;
    bool is_yaw_reached_flag = true; //we don't check yaw reach for rover (nonholonomic vehicle)

    //check position reach condition
    if (dist_xy_sq<nav_acc_rad_xy*nav_acc_rad_xy){
        is_position_reached_flag = true;
    }

    if (current_wpindex == waypoints.size()-1 && current_state.armed)
        ROS_INFO_ONCE("Heading for the last waypoint");

    if (is_position_reached_flag && is_yaw_reached_flag){

        if (current_wpindex < waypoints.size()-1)
            current_wpindex++;
        else{
            _flag_last_wp_reached = true;

            _mission_phase = ROVER_MISSION_PHASE::VEL_PHASE;

            _phase_entry_timestamp = ros::Time::now();
            ROS_INFO("The waypoint mission in POS_PHASE is finished, switch to VEL_PHASE");
        }

    }

}
