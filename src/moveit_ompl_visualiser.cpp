#include "moveit_ompl_visualiser.h"

#include <std_msgs/Bool.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
/*
 You can store ompl data with:
 ```cpp
    ompl::base::PlannerData ompl_data_(simple_setup->getSpaceInformation());

    ompl::base::PlannerDataStorage ompl_data_storage_;
    ompl_data_storage_.store(ompl_data_,"sampled_states");
 ```

 You can then retrieve ompl data (possibly with other proces as well) with:
 ```cpp
    ompl::base::PlannerData sampledData(temp_planning_context->getOMPLSimpleSetup()->getSpaceInformation());

    // load the data from binary file and store it as a PlannerData
    ompl::base::PlannerDataStorage stored_sampled_Data;
    stored_sampled_Data.load(filename_to_load, sampledData);
 ```
 * */



#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/remote_control.h>


std::default_random_engine get_rand((unsigned int)time(0));


namespace moveit_ompl_visualiser {

    inline std_msgs::ColorRGBA color(float r, float g, float b, float a=1.0f) {
        std_msgs::ColorRGBA msg;
        msg.r = r;
        msg.g = g;
        msg.b = b;
        msg.a = a;
        return msg;
    }

    // a pointer to process incoming planning context

    static void cb_enable_vis_(const std_msgs::Bool::ConstPtr &msg);


    ros::NodeHandle nh_(moveit_ompl_visualiser::topic_ns);
//    std::shared_ptr<ros::Subscriber> sub_ = std::make_shared<ros::Subscriber>(
//            nh_.subscribe(moveit_ompl_visualiser::topic_ns + "/enabled", 1000, cb_enable_vis_));
    ros::Subscriber sub_ = nh_.subscribe("enabled", 100, cb_enable_vis_);

    std::shared_ptr<ros::Publisher> marker_pub;
    std::shared_ptr<visualization_msgs::MarkerArray> last_vis_arr_;


    ///////////////////////////////////////////////////////////////////
    ompl_interface::ModelBasedPlanningContext *context_;
    std::shared_ptr<ompl_interface::ModelBasedStateSpace> ompl_state_space_;
    std::string LINK_NAME_;
    std::shared_ptr<robot_state::RobotState> robot_state_;
    ///////////////////////////////////////////////////////////////////
    std::string REFERENCE_FRAME_;
    ///////////////////////////////////////////////////////////////////

    std::vector<visualization_msgs::Marker> displayed_mks_;
    std::vector<visualization_msgs::Marker> displayed_sol_;

    bool enabled = false;
    static void cb_enable_vis_(const std_msgs::Bool::ConstPtr &msg) {
        enabled = msg->data;
        ROS_INFO("moveit_ompl_visualiser -> ", msg->data ? "enabled" : "disabled");
//        nh_.getParam("link_name", LINK_NAME_);
//        nh_.getParam("/moveit_visualize_data_node/moveit_ompl_visualiser/link_name", LINK_NAME_);
        ros::param::get("/moveit_visualize_data_node/moveit_ompl_visualiser/link_name", LINK_NAME_);
    }

    void set_context(ompl_interface::ModelBasedPlanningContext *context) {
        context_ = context;
        ompl_state_space_ = context->getOMPLStateSpace();

        const auto &planning_scene = context->getPlanningScene();
        robot_state_ = std::make_shared<robot_state::RobotState>(planning_scene->getCurrentState());
        /////////////
        REFERENCE_FRAME_ = planning_scene->getPlanningFrame();
        /////////////


//        last_vis_arr_ = std::make_shared<visualization_msgs::MarkerArray>();

        clear_markers();
    }

    bool check_guard(bool check_context=true) {
        if (!enabled)
            return false;
        if (check_context && !context_) {
            ROS_WARN("Context has not been set yet! Ignoring...");
            return false;
        }
//        if (sub_->getNumPublishers() < 1)
//            return;
        if (!marker_pub) {
            // Create the message publisher and advertise,
            ros::NodeHandle nh_ = ros::NodeHandle();
            marker_pub = std::make_shared<ros::Publisher>(std::move(
                    nh_.advertise<visualization_msgs::MarkerArray>(
                            moveit_ompl_visualiser::marker_array_topic, 10)));
            // wait some time for rviz to successfully connects to the channel.
            sleep(1);
        }
        return true;
    }

    visualization_msgs::Marker marker_template_(std::string ns="template marker") {
        visualization_msgs::Marker mk;
        mk.header.stamp = ros::Time();
        mk.header.frame_id = REFERENCE_FRAME_;
        mk.ns = std::move(ns);
        mk.id = get_rand();
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.lifetime = ros::Duration(0);
        return mk;
    }

    void pub_marker_to_markerArray(const std::shared_ptr<ros::Publisher> &pub_,
                                   const visualization_msgs::Marker &mk) {
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(mk);
        pub_->publish(arr);
    }

    inline geometry_msgs::Point vertex_state_to_position_msg(
            const ompl::base::State *state,
            const std::shared_ptr<ompl_interface::ModelBasedStateSpace> &ompl_state_space,
            robot_state::RobotState &robot_state,
            const std::string &LINK_NAME) {
        // each vertex of the graph contains the complete state of the robot, this state needs to be converted into an Movit robot state
        ompl_state_space->copyToRobotState(robot_state, state);

        // the translational (position) component of the desired link is the one we are interested
        const Eigen::Isometry3d &end_effector_state = robot_state.getGlobalLinkTransform(
                LINK_NAME);
        const Eigen::Vector3d &position = end_effector_state.translation();

        geometry_msgs::Point p_vi;
        p_vi.x = position.x();
        p_vi.y = position.y();
        p_vi.z = position.z();
        return p_vi;
    }

    void show_node(ompl::base::State *state) {
        ROS_INFO("Requestingn to show node");
        if (!check_guard())
            return;
        ROS_INFO("Preparing");
        geometry_msgs::Point p_vi = vertex_state_to_position_msg(
                state, ompl_state_space_, *robot_state_, LINK_NAME_);


        double NODE_SIZE = 0.0008;
        double EDGE_SIZE = 0.0008;
        NODE_SIZE = 0.1;
        EDGE_SIZE = 0.005;

        // fill in a new visualization message (MarkerArray)
        visualization_msgs::Marker mk = marker_template_("sampled vertex");
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position = p_vi;
        mk.scale.x = mk.scale.y = mk.scale.z = NODE_SIZE;
        mk.color = color(0, 1, .8);
        displayed_mks_.push_back(mk);  // book keeping

        pub_marker_to_markerArray(marker_pub, mk);
    }

    void show_edge(ompl::base::State *state1, ompl::base::State *state2, double EDGE_SIZE) {
        if (!state1 || !state2) return;
        if (!check_guard())
            return;

        visualization_msgs::Marker line_list = marker_template_("sampled edges");
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.scale.x = EDGE_SIZE;
        line_list.color = color(0, .5, .5);

        line_list.points.push_back(vertex_state_to_position_msg(
                state1, ompl_state_space_, *robot_state_, LINK_NAME_));
        line_list.points.push_back(vertex_state_to_position_msg(
                state2, ompl_state_space_, *robot_state_, LINK_NAME_));

        pub_marker_to_markerArray(marker_pub, line_list);

        displayed_mks_.push_back(line_list);  // book keeping
    }

    void show_solution(std::vector<ompl::base::State *> states, double EDGE_SIZE) {
        if (!check_guard())
            return;
        // always clear previous solutions
        for (auto &sol: displayed_sol_) {
            sol.action = visualization_msgs::Marker::DELETE;
            pub_marker_to_markerArray(marker_pub, sol);
        }
        displayed_sol_.clear();

        visualization_msgs::Marker line_list = marker_template_("sampled edges");
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.scale.x = EDGE_SIZE;
        line_list.color = color(0, .95, .2);

        for (auto &state: states)
            line_list.points.push_back(vertex_state_to_position_msg(
                    state, ompl_state_space_, *robot_state_, LINK_NAME_));

        pub_marker_to_markerArray(marker_pub, line_list);

        displayed_sol_.push_back(line_list);  // book keeping
    }


    void clear_markers() {
        if (!check_guard())
            return;

        visualization_msgs::MarkerArray arr;
        for (auto & m: displayed_mks_) {
            m.action = visualization_msgs::Marker::DELETE;
            arr.markers.push_back(m);
        }
        marker_pub->publish(arr);
        displayed_mks_.clear();
    }





    inline void display_node_data(ompl_interface::ModelBasedPlanningContext *context,
                                  const std::string &LINK_NAME) {

//        static const std::string LINK_NAME = "panda_link8";
        // specify the reference frame of the marker (header.frame_id)
//        std::string REFERENCE_FRAME = "world";
        // specify the Planning Group, that had been used to generate the samples
//        std::string PLANNING_GROUP_FOR_ANALYSIS = "panda_arm";
        // specify the state space factory type (can be trial and error or just print out the factory type that had been used during the last motion plan request)
//        std::string STATE_SPACE_MODEL = "JointModel";


        // use the OMPL data storage functionality to store all the sampled states in graphml and binary format
        // IMPORTANT: the data is only properly stored, if no parallel plan is used (at the moment)
        auto simple_setup = context->getOMPLSimpleSetup();
        const auto &planning_scene = context->getPlanningScene();
        std::string REFERENCE_FRAME = planning_scene->getPlanningFrame();
        REFERENCE_FRAME = "world";


        ompl::base::PlannerData sampledData(simple_setup->getSpaceInformation());
        simple_setup->getPlannerData(sampledData);
        //get planning scene
        robot_state::RobotState robot_state = planning_scene->getCurrentState();

        ROS_INFO("GK: planning frame %s", planning_scene->getPlanningFrame().c_str());


        last_vis_arr_ = std::make_shared<visualization_msgs::MarkerArray>();


        unsigned int numV = sampledData.numVertices();
        unsigned int numE = sampledData.numEdges();
        ROS_INFO(
                "Inside data visualizing tool: total number of vertices to be displaced: %i \n",
                numV);

        const std::shared_ptr<ompl_interface::ModelBasedStateSpace> ompl_state_space = context->getOMPLStateSpace();

        double NODE_SIZE = 0.0008;
        double EDGE_SIZE = 0.0008;
        NODE_SIZE = 0.01;
        EDGE_SIZE = 0.005;

        for (unsigned int i = 0; i < numV; ++i) {

            const ompl::base::State *state = sampledData.getVertex(i).getState();
            geometry_msgs::Point p_vi = vertex_state_to_position_msg(
                    state, ompl_state_space, robot_state, LINK_NAME);

            // for each vertex check also each edge which is going out of this vertex
            std::vector<unsigned int> edgeList;
            int edgeList_size = sampledData.getEdges(i, edgeList);
            ROS_DEBUG_NAMED("GK",
                            "GK:ompl_planner_manager %d number of edges for vertex %d",
                            edgeList_size, i);
            for (int j = 0; j < edgeList_size; j++) {
                visualization_msgs::Marker line_list;
                line_list.header.stamp = ros::Time();
                line_list.header.frame_id = REFERENCE_FRAME;
                line_list.ns = "created edges";
                line_list.id = get_rand();
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.scale.x = EDGE_SIZE;
                line_list.pose.orientation.w = 1.0;
                line_list.points.push_back(p_vi);
                line_list.color.r = 1.0;
                line_list.color.a = 1.0;

                const ompl::base::State *state_j = sampledData.getVertex(
                        edgeList[j]).getState();
                geometry_msgs::Point p_vj = vertex_state_to_position_msg(
                        state_j, ompl_state_space, robot_state, LINK_NAME);

                line_list.points.push_back(p_vj);
                last_vis_arr_->markers.push_back(line_list);

                ROS_DEBUG_NAMED("GK",
                                "GK pose for connected vertex number %d towards pose: x:%.5f, y: %.5f, z:%.5f",
                                i, p_vj.x, p_vj.y, p_vj.z);
            }


            // fill in a new visualization message (MarkerArray)
            visualization_msgs::Marker mk = marker_template_("sampled vertex");
            mk.type = visualization_msgs::Marker::SPHERE;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position = p_vi;
            std_msgs::ColorRGBA color_node_green = color(0, .8, 0);
            mk.color = color_node_green;
            last_vis_arr_->markers.push_back(mk);

            ROS_INFO(
                    "GK pose for vertex number %d with pose: x:%.5f, y: %.5f, z:%.5f",
                    i, p_vi.x, p_vi.y, p_vi.z);

        }
        // publish message to visualize the sampled states
        // wait until someone is actually listening.
        marker_pub->publish(*last_vis_arr_);
        std::cout << *last_vis_arr_ << std::endl;

        ROS_INFO("OMPL Planner Manager: finished visualizing sampled position");
        ROS_INFO("published in %s",
                 moveit_ompl_visualiser::marker_array_topic.c_str());
    }



    void add_planned_context(ompl_interface::ModelBasedPlanningContext *context) {

        if (!check_guard(false))
            return;

        // remove previously displayed markers
        if (last_vis_arr_) {
            for (auto &&marker: last_vis_arr_->markers)
                marker.action = visualization_msgs::Marker::DELETE;
            marker_pub->publish(*last_vis_arr_);
        }

        static const std::string LINK_NAME = "panda_link8";
        moveit_visual_tools::MoveItVisualTools visual_tools(LINK_NAME);
        rviz_visual_tools::RemoteControlPtr remote_ctrl = visual_tools.getRemoteControl();


//        while(!remote_ctrl->getStop()) {
        display_node_data(context, LINK_NAME);
        visual_tools.prompt(
                "Press 'next' in the RvizVisualToolsGui window to visualize the next samples and delete the current markers");
        visual_tools.deleteAllMarkers();
//        }

    }


};
