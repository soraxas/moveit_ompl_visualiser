#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include "moveit_ompl_visualiser/moveit_ompl_visualiser.h"

#include <signal.h>
#include <unistd.h>

class MoveitOmplVisualiser;
// a static visualiser pointer
std::shared_ptr<MoveitOmplVisualiser> visualiser;

class MoveitOmplVisualiser {
public:

    ros::NodeHandle nh_ = ros::NodeHandle(moveit_ompl_visualiser::topic_ns);
    ros::Publisher pub_;

    std_msgs::Bool enable_flag;

    ~MoveitOmplVisualiser() {
        pub_.shutdown();
    }

    void shutdown() {
        enable_flag.data = false;
        pub_.publish(enable_flag);
        // if there's no sleep, the last msg wouldn't be delivered
        sleep(1);
    }

    void _connected_cb(const ros::SingleSubscriberPublisher &pub) {
        pub_.publish(enable_flag);
        ROS_INFO("NEW SUB");
    }

public:
    MoveitOmplVisualiser(int argc, char **argv) {
        pub_ = nh_.advertise<std_msgs::Bool>("enabled", 10,
                                             [this](auto &&PH1) { _connected_cb(PH1); },
                                             [this](auto &&PH1) { _connected_cb(PH1); },
                                             nullptr,
                                             true
        );
        enable_flag.data = true;
        sleep(1);
        pub_.publish(enable_flag);
    }
};

void mySigintHandler(int sig) {
    visualiser->shutdown();
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_ompl_visualiser_node",
              ros::init_options::NoSigintHandler);

    visualiser = std::make_shared<MoveitOmplVisualiser>(argc, argv);
    // handler that properly shutdown the visualiser
    signal(SIGINT, mySigintHandler);

    ros::spin();
    return 0;
}
