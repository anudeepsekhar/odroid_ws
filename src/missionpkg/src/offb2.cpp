#include <iostream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <math.h>

class ifor_drone
{
public:
    ifor_drone() {}
    ~ifor_drone() {}

public:
    void init();

    void land();

    void move_to_loc();

    bool is_target_pos();

    void set_mission(int repeat);

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void move_x_cb(const std_msgs::String::ConstPtr& msg);
    void move_y_cb(const std_msgs::String::ConstPtr& msg);
    void move_z_cb(const std_msgs::String::ConstPtr& msg);

public:
    ros::NodeHandle     n;
    ros::Subscriber     state_sub;
    ros::Subscriber     location_sub;

    ros::Subscriber     move_x_sub;
    ros::Subscriber     move_y_sub;
    ros::Subscriber     move_z_sub;

    ros::Publisher      local_pos_pub;

    ros::ServiceClient  arming_cl;
    ros::ServiceClient  disarming_cl;
    ros::ServiceClient  set_mode_cl;
    ros::ServiceClient  land_cl;

    mavros_msgs::State  current_state;

    geometry_msgs::PoseStamped     target_loc;
    geometry_msgs::PoseStamped     local_loc;

};

void ifor_drone::init()
{
    state_sub      = n.subscribe<mavros_msgs::State>("mavros/state", 10, &ifor_drone::state_cb, this);
    location_sub   = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &ifor_drone::local_pos_cb, this);

    move_x_sub     = n.subscribe("ifor_drone/x_pos", 10, &ifor_drone::move_x_cb, this);
    move_y_sub     = n.subscribe("ifor_drone/y_pos", 10, &ifor_drone::move_y_cb, this);
    move_z_sub     = n.subscribe("ifor_drone/z_pos", 10, &ifor_drone::move_z_cb, this);

    local_pos_pub    = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
       
    arming_cl      = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    disarming_cl   = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/disarming");
    set_mode_cl    = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    land_cl        = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    ros::Rate rate_20(20.0);

    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate_20.sleep();
    }    
}

void ifor_drone::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;

    std::cout << "\n[ifor] state_cb(), -------------------------------";
    std::cout << "\n          current_state.connected = " << ((current_state.connected) ? "OK!" : "Not yet!");
    std::cout << "\n          current_state.armed = " << ((current_state.armed ) ? "OK!" : "Not yet!");
    std::cout << "\n          current_state.guided = " << ((current_state.guided) ? "OK!" : "Not yet!");
    std::cout << "\n          current_state.mode = " << current_state.mode;
    std::cout << "\n          target_loc.pose.position.x = " << target_loc.pose.position.x;
    std::cout << "\n          target_loc.pose.position.y = " << target_loc.pose.position.y;
    std::cout << "\n          target_loc.pose.position.z = " << target_loc.pose.position.z;
    std::cout << "\n[ifor] -----------------------------------------\n";
}

void ifor_drone::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_loc = *msg;
/*
    std::cout << "\n[ifor] cbLocalPosition(), -----------";
    std::cout << "\n          local_loc.pose.position.x = " << local_loc.pose.position.x;
    std::cout << "\n          local_loc.pose.position.y = " << local_loc.pose.position.y;
    std::cout << "\n          local_loc.pose.position.z = " << local_loc.pose.position.z;
    std::cout << "\n[ifor] ------------------------\n";
*/
}

void ifor_drone::move_x_cb(const std_msgs::String::ConstPtr& msg)
{
  std::string::size_type sz;   

  int pos = std::stoi (msg->data, &sz);

  target_loc.pose.position.x = pos;
}

void ifor_drone::move_y_cb(const std_msgs::String::ConstPtr& msg)
{
  std::string::size_type sz; 

  int pos = std::stoi (msg->data, &sz);

  target_loc.pose.position.y = pos;
}

void ifor_drone::move_z_cb(const std_msgs::String::ConstPtr& msg)
{
  std::string::size_type sz;  

  int pos = std::stoi (msg->data, &sz);

  target_loc.pose.position.z = pos;
}

bool ifor_drone::is_target_pos()
{
     if ( abs(target_loc.pose.position.x - local_loc.pose.position.x) < 0.03 &&
          abs(target_loc.pose.position.y - local_loc.pose.position.y) < 0.03 &&
          abs(target_loc.pose.position.z - local_loc.pose.position.z) < 0.03)
     {
        //  std::cout << "\n[ifor] Here is the target position!\n";
        return true;
     }

    return false;   
}

void ifor_drone::move_to_loc()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    while (ros::ok()) {
        
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_cl.call(offb_set_mode) &&
                offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_cl.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }

                last_request = ros::Time::now();
            }
        }
        
        if (is_target_pos())
            return;
        else
            local_pos_pub.publish(target_loc);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void ifor_drone::set_mission(int repeat)
{
    std::cout << "\n[ifor] test set_mission!\n";

    ros::Rate rate_50(50.0);

    geometry_msgs::PoseStamped mission_pos_arr[6];

    mission_pos_arr[0].pose.position.x = 1;
    mission_pos_arr[0].pose.position.y = 1;
    mission_pos_arr[0].pose.position.z = 1.75f;

    mission_pos_arr[1].pose.position.x = -1;
    mission_pos_arr[1].pose.position.y = 1;
    mission_pos_arr[1].pose.position.z = 1.75f;

    mission_pos_arr[2].pose.position.x = -1;
    mission_pos_arr[2].pose.position.y = -1;
    mission_pos_arr[2].pose.position.z = 1.75f;

    mission_pos_arr[3].pose.position.x = 1;
    mission_pos_arr[3].pose.position.y = -1;
    mission_pos_arr[3].pose.position.z = 1.75f;

    mission_pos_arr[4].pose.position.x = 1;
    mission_pos_arr[4].pose.position.y = 1;
    mission_pos_arr[4].pose.position.z = 1.75f;

    mission_pos_arr[5].pose.position.x = 0;
    mission_pos_arr[5].pose.position.y = 0;
    mission_pos_arr[5].pose.position.z = 1.75f;
  
    for (int count = 0; count < repeat && ros::ok(); count++)
    {
        target_loc.pose.position.x = mission_pos_arr[0].pose.position.x;
        target_loc.pose.position.y = mission_pos_arr[0].pose.position.y;
        target_loc.pose.position.z = mission_pos_arr[0].pose.position.z;

        int nPosIndex = 0;
        while (ros::ok())
        {
            if (!is_target_pos())
            {
                move_to_loc();
            }
            else
            {
                if (++nPosIndex == 30)
                    break;

                target_loc.pose.position.x = mission_pos_arr[nPosIndex].pose.position.x;
                target_loc.pose.position.y = mission_pos_arr[nPosIndex].pose.position.y;
                target_loc.pose.position.z = mission_pos_arr[nPosIndex].pose.position.z;
            }

            rate_50.sleep();
        }
    }
}


void ifor_drone::land()
{
    mavros_msgs::CommandTOL land_cmd;

    land_cmd.request.altitude = 1.75f;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;

    if(land_cl.call(land_cmd)){
        ROS_INFO("land_cmd send ok %d", land_cmd.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard2");
    
    std::thread t([]() {
         ros::AsyncSpinner spinner(1); 
         spinner.start();
         ros::waitForShutdown();
    });

    ifor_drone drone;
    
    ros::Rate rate_50(50.0);

    drone.init();

    drone.set_mission(1);

    rate_50.sleep();

    drone.land();

    return 0;
}
