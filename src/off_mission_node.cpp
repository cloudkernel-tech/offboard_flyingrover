/**
 * @file off_mission_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>

#include <tf/transform_datatypes.h>


//variables
static std::vector<geometry_msgs::PoseStamped> waypoints;
static int current_wpindex = 0;//waypoint index starts from zero
static mavros_msgs::State current_state;
static mavros_msgs::ExtendedState current_extendedstate;
static geometry_msgs::PoseStamped current_local_pos;
static mavros_msgs::RCIn rcinput;

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

    //subscriptions, publications and services
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pose_cb);
    //ros::Subscriber local_vel_sub  = nh.subscribe("mavros/local_position/velocity", 10, local_vel_cb);    

    ros::Subscriber rc_input_sub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in", 5, rc_input_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    /* param loading*/
    ros::NodeHandle private_nh("~");

    // load yaml files
    XmlRpc::XmlRpcValue wp_list;
    private_nh.getParam("waypoints",wp_list);
    initTagetVector(wp_list);
    ROS_INFO("waypoint yaml loaded in main program");

    // simulation flag
    int simulation_flag = 0;
    private_nh.param("simulation_flag", simulation_flag, 0);


    geometry_msgs::PoseStamped pose; //pose to be passed to fcu

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();

    bool flag_poweroff_rc_en = false;

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

        pose = waypoints.at(current_wpindex);

        local_pos_pub.publish(pose);

        updateWaypointIndex();

        //disarm when landed and the vehicle is heading for the last waypoint
        if (current_wpindex == waypoints.size()-1
                && current_extendedstate.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_LANDING)
        {
            if( current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(disarm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
                last_request = ros::Time::now();

                flag_poweroff_rc_en = true;
            }

        }

        //power off by rc aux1 channel (CH7) after landing and disarmed
        if (flag_poweroff_rc_en)
        {
            if (rcinput.channels[6]>1600)
            {
                ROS_INFO("Shutdown by Rc");
                system("shutdown -P now");
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
    // index will be updated when the position is reached
    float dx = current_local_pos.pose.position.x - waypoints.at(current_wpindex).pose.position.x;
    float dy = current_local_pos.pose.position.y - waypoints.at(current_wpindex).pose.position.y;
    float dz = current_local_pos.pose.position.z - waypoints.at(current_wpindex).pose.position.z;

    float dist_sq = dx*dx+dy*dy+dz*dz;

    if (dist_sq<0.04f)
    {
        if (current_wpindex < waypoints.size()-1)
            current_wpindex++;
        else
            ROS_INFO_THROTTLE(2, "The waypoint mission is finished");
    }

    if (current_wpindex == waypoints.size()-1 && current_state.armed)
        ROS_INFO_THROTTLE(2, "Heading for the last waypoint");


}
