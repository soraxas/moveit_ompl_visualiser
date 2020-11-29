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

    namespace rvt = rviz_visual_tools;

    inline std_msgs::ColorRGBA color(double r, double g, double b, double a=1.0) {
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
    static void cb_enable_vis_(const std_msgs::Bool::ConstPtr &msg=nullptr) {
        if (msg) {
            enabled = msg->data;
        } else {
            enabled = true;  // default to true
        }
        ROS_INFO_NAMED("moveit_ompl_visualiser", "moveit_ompl_visualiser -> %s", enabled ? "enabled" : "disabled");
        ros::param::get("/moveit_visualize_data_node/moveit_ompl_visualiser/link_name", LINK_NAME_);
    }

    void set_context(ompl_interface::ModelBasedPlanningContext *context) {
        context_ = context;
        /////////////
        ompl_state_space_ = context->getOMPLStateSpace();
        const auto &planning_scene = context->getPlanningScene();
        robot_state_ = std::make_shared<robot_state::RobotState>(planning_scene->getCurrentState());
        REFERENCE_FRAME_ = planning_scene->getPlanningFrame();
        /////////////
        clear_markers();
    }

    bool check_guard(bool check_context=true) {
        if (!enabled)
            return false;
        if (check_context && !context_) {
            ROS_WARN("Context has not been set yet! Ignoring...");
            return false;
        }
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

    inline void pub_marker_to_markerArray(const std::shared_ptr<ros::Publisher> &pub_,
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

    void show_node(ompl::base::State *state, double NODE_SIZE, double time_to_live, double r, double g, double b) {
        if (!check_guard())
            return;
        geometry_msgs::Point p_vi = vertex_state_to_position_msg(
                state, ompl_state_space_, *robot_state_, LINK_NAME_);

        // fill in a new visualization message (MarkerArray)
        visualization_msgs::Marker mk = marker_template_("sampled vertex");
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position = p_vi;
        mk.scale.x = mk.scale.y = mk.scale.z = NODE_SIZE;
        mk.color = color(r, g, b);
        mk.lifetime = ros::Duration(time_to_live);
        if (time_to_live == 0)
            displayed_mks_.push_back(mk);  // book keeping

        pub_marker_to_markerArray(marker_pub, mk);
    }

    void show_sampled_state(ompl::base::State * state) {
        show_node(state, 0.02, 2,1, 0, 0);
    }

    void show_sampled_robot_state(ompl::base::State * state) {
        /*
         * NOTE You must ADD a new display in RVIZ to view the sampled robot state,
         * under the "display_robot_state" channel.
         * */
        // copy state to robot state
        vertex_state_to_position_msg(state, ompl_state_space_, *robot_state_, LINK_NAME_);
        // copy it to prevent changing
        auto robot_state1 = std::make_shared<moveit::core::RobotState>(*robot_state_);

        /////////////////////////////////////////////////////
        moveit_visual_tools::MoveItVisualTools visual_tools(LINK_NAME_);
        visual_tools.loadRobotStatePub("/display_robot_state");
        visual_tools.enableBatchPublishing();

        const std::string PLANNING_GROUP = "panda_arm";
        const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(PLANNING_GROUP);
        visual_tools.publishRobotState(robot_state1);

        visual_tools.trigger();
        ROS_INFO("post");
        /////////////////////////////////////////////////////
    }

    void show_sampled_robot_state_traj(ompl::base::State * state1, ompl::base::State * state2) {
        /*
         * NOTE You must ADD a new display in RVIZ to view the sampled robot state,
         * under the "display_robot_state" channel.
         * */
        // copy state to robot state
        vertex_state_to_position_msg(state1, ompl_state_space_, *robot_state_, LINK_NAME_);
        auto robot_state1 = std::make_shared<moveit::core::RobotState>(*robot_state_);
        //
        vertex_state_to_position_msg(state2, ompl_state_space_, *robot_state_, LINK_NAME_);
        auto robot_state2 = std::make_shared<moveit::core::RobotState>(*robot_state_);

        /////////////////////////////////////////////////////
        moveit_visual_tools::MoveItVisualTools visual_tools(LINK_NAME_);
        visual_tools.loadRobotStatePub("/display_robot_state");
        visual_tools.enableBatchPublishing();

        const std::string PLANNING_GROUP = "panda_arm";
        const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(PLANNING_GROUP);


        robot_trajectory::RobotTrajectory traj(context_->getRobotModel(), PLANNING_GROUP);
        traj.addSuffixWayPoint(robot_state1, .5);
        traj.addSuffixWayPoint(robot_state2, .5);

        visual_tools.publishTrajectoryLine(traj, joint_model_group);

        visual_tools.trigger();
        /////////////////////////////////////////////////////
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

        std::shared_ptr<geometry_msgs::Point> _last_displayed_pt;
        for (auto &state: states) {
            auto display_state = vertex_state_to_position_msg(
                        state, ompl_state_space_, *robot_state_, LINK_NAME_);
            if (_last_displayed_pt) {
                line_list.points.push_back(*_last_displayed_pt);
                line_list.points.push_back(display_state);
            }
            _last_displayed_pt = std::make_shared<geometry_msgs::Point>(
                    display_state);
        }

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

    void show_planner_data(ompl_interface::ModelBasedPlanningContext *context) {
        if (!context) {  // use default context
            if (!check_guard())
                return;
            context = context_;
        }
        size_t item_num_to_remove = 0;
        auto to_be_removed_ = last_vis_arr_;
        if (to_be_removed_) {
            // remove previously displayed markers
            for (auto &&marker: to_be_removed_->markers) {
                marker.action = visualization_msgs::Marker::DELETE;
                item_num_to_remove += 1;
            }

        }
        last_vis_arr_ = std::make_shared<visualization_msgs::MarkerArray>();


        auto simple_setup = context->getOMPLSimpleSetup();
        const auto &planning_scene = context->getPlanningScene();
        std::string REFERENCE_FRAME = planning_scene->getPlanningFrame();

        ompl::base::PlannerData plannerData(simple_setup->getSpaceInformation());
        simple_setup->getPlannerData(plannerData);
        //get planning scene
        robot_state::RobotState robot_state = planning_scene->getCurrentState();



        unsigned int numV = plannerData.numVertices();
        unsigned int numE = plannerData.numEdges();

        const std::shared_ptr<ompl_interface::ModelBasedStateSpace> ompl_state_space = context->getOMPLStateSpace();

        double NODE_SIZE = 0.0008;
        double EDGE_SIZE = 0.0008;
        NODE_SIZE = 0.01;
        EDGE_SIZE = 0.005;

        for (unsigned int i = 0; i < numV; ++i) {

            const ompl::base::State *state = plannerData.getVertex(i).getState();
            geometry_msgs::Point p_vi = vertex_state_to_position_msg(
                    state, ompl_state_space, robot_state, LINK_NAME_);

            // for each vertex check also each edge which is going out of this vertex
            std::vector<unsigned int> edgeList;
            int edgeList_size = plannerData.getEdges(i, edgeList);

//            std_msgs::ColorRGBA color_node_= color(0, .8, 0);
            std_msgs::ColorRGBA color_roadmap= color(0.6, .8, 1.);

            for (int j = 0; j < edgeList_size; j++) {
                visualization_msgs::Marker line_list = marker_template_("created edges");
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.scale.x = EDGE_SIZE;
                line_list.color = color_roadmap;

                const ompl::base::State *state_j = plannerData.getVertex(
                        edgeList[j]).getState();
                geometry_msgs::Point p_vj = vertex_state_to_position_msg(
                        state_j, ompl_state_space, robot_state, LINK_NAME_);

                line_list.points.push_back(p_vi);
                line_list.points.push_back(p_vj);
                last_vis_arr_->markers.push_back(line_list);
            }

            // fill in a new visualization message (MarkerArray)
            visualization_msgs::Marker mk = marker_template_("sampled vertex");
            mk.type = visualization_msgs::Marker::SPHERE;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position = p_vi;
            mk.scale.x = mk.scale.y = mk.scale.z = NODE_SIZE;
            mk.color = color_roadmap;
            last_vis_arr_->markers.push_back(mk);


        }
            // publish message to visualize the sampled states
        // wait until someone is actually listening.
        marker_pub->publish(*last_vis_arr_);
        if (to_be_removed_) {
            marker_pub->publish(*to_be_removed_);
            to_be_removed_->markers.clear();
        }

//        // remove from the message
//        if (item_num_to_remove > 0) {
//            auto &_mks = last_vis_arr_->markers;
//            _mks.erase(_mks.begin(), _mks.begin() + item_num_to_remove);
//        }
    }


    void display_planned_context(ompl_interface::ModelBasedPlanningContext *context) {
        if (!enabled && ros::param::has("/moveit_visualize_data_node/moveit_ompl_visualiser/link_name")) {
            // a workaround to make this periodically (every time it finishes a sol) check for whether
            // vis node is present.
            cb_enable_vis_();
            enabled = true;
        }
        if (!check_guard(false))
            return;

        // remove previously displayed markers
        if (last_vis_arr_) {
            for (auto &&marker: last_vis_arr_->markers)
                marker.action = visualization_msgs::Marker::DELETE;
            marker_pub->publish(*last_vis_arr_);
        }

        moveit_visual_tools::MoveItVisualTools visual_tools(LINK_NAME_);
        rviz_visual_tools::RemoteControlPtr remote_ctrl = visual_tools.getRemoteControl();


//        while(!remote_ctrl->getStop()) {
        show_planner_data(context);

        bool pause_at_sol;
        ros::param::get("/moveit_visualize_data_node/moveit_ompl_visualiser/pause_at_solution", pause_at_sol);
        if (pause_at_sol)
            visual_tools.prompt(
                    "Press 'next' in the RvizVisualToolsGui window to visualize the next samples and delete the current markers");
        visual_tools.deleteAllMarkers();
//        }
    }

};
