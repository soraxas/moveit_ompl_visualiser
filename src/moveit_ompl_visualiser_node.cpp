#include <ros/ros.h>

// MoveIt
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/planning_interface/planning_interface.h>
//#include <moveit/planning_scene/planning_scene.h>
//
//#include <moveit/kinematic_constraints/utils.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <rviz_visual_tools/remote_control.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// REMARK: This node should run properly with the adapted moveit branch
/// IMPORTANT: the OMPL data is only properly stored if no parallel plan is used, ony if the PlanningAttempts=1 is set, you can
/// visualize the data
/// This node can be launched with "roslaunch moveit_ompl_visualiser sampling_data_analysis.launch" and everytime "next"
/// is pressed in the RvizVisualToolsGui the last stored planner data is visualized.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <std_msgs/Bool.h>
#include "moveit_ompl_visualiser.h"
//#include "moveit_ompl_visualiser/moveit_ompl_visualiser.h"

//#include "moveit_ompl_visualiser.h"
//#include <ompl/base/PlannerData.h>
//#include <ompl/base/PlannerDataStorage.h>

#include <signal.h>
#include <unistd.h>
#include <atomic>

std::atomic<bool> quit(false);    // signal flag

void mySigintHandler(int sig) {
    quit.store(true);
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    std::cout << "Shuting down..." << std::endl;

    // All the default sigint handler does is call shutdown()
//  ros::shutdown();
}


class MoveitOmplVisualiser {
private:

public:

    ros::NodeHandle nh_ = ros::NodeHandle(moveit_ompl_visualiser::topic_ns);
    ros::Publisher pub_;

    std_msgs::Bool enable_flag;


    static void chatterCallback(const std_msgs::Bool::ConstPtr &msg) {
        if (msg->data)
            ROS_INFO("I heard: yes");
        else
            ROS_INFO("I heard: no");
    }

    ~MoveitOmplVisualiser() {
        pub_.shutdown();
    }

    void shutdown() {
        moveit_ompl_visualiser::processor = nullptr;
        enable_flag.data = false;
        pub_.publish(enable_flag);
        // if no sleep, the last msg would be delivered
        sleep(1);
    }

public:
    MoveitOmplVisualiser(int argc, char **argv) {
        pub_ = nh_.advertise<std_msgs::Bool>("enabled", 10, true);
        enable_flag.data = true;
        pub_.publish(enable_flag);
    }
};


int main(int argc, char **argv) {
    // Override SIGINT handler
    ros::init(argc, argv, "moveit_ompl_visualiser_node",
              ros::init_options::NoSigintHandler);

    signal(SIGINT, mySigintHandler);

    {
        MoveitOmplVisualiser visualiser(argc, argv);


//        visualiser.nh_.setParam("link_name", "my_string");


        //    while(ros::ok) {
        while (!quit.load()) {
            ros::spinOnce();
        }
        visualiser.shutdown();
    }

//    ros::shutdown();
    return 0;
}
