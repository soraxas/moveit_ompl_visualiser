#ifndef ROS_MASTER_MOVEIT_OMPL_VISUALISER_H
#define ROS_MASTER_MOVEIT_OMPL_VISUALISER_H

#include <moveit/ompl_interface/model_based_planning_context.h>


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

namespace moveit_ompl_visualiser {
    using planning_context_process_ptr = void (*)(ompl_interface::ModelBasedPlanningContext *);
    // a pointer to process incoming planning context


    const std::string topic_ns = "moveit_ompl_visualiser";
//    const std::string marker_array_topic = topic_ns + "/rviz_visual_tools";
//    const std::string marker_array_topic ="/rviz_visual_tools";
    const std::string marker_array_topic ="rviz_visual_tools";
//    const std::string marker_array_topic ="/moveit_ompl_visualiser_markers";

//    const char *topic_ns = "/moveit_ompl_visualiser";

//    // Added visualization function for sampled states by Gabriel Koenig
//    void visualizeSampledStates(const ompl_interface::ModelBasedPlanningContext *context,
//                              const std::string& PLANNING_GROUP,
//                              const std::string& LINK_NAME,
//                              const std::string& REFERENCE_FRAME,
//                              const std::string& state_space_model,
//                              const char *filename_to_load);

    void set_context(ompl_interface::ModelBasedPlanningContext *context);

    void show_planner_data(ompl_interface::ModelBasedPlanningContext *context=nullptr);
    void display_planned_context(ompl_interface::ModelBasedPlanningContext *context);
    void show_node(ompl::base::State *, double NODE_SIZE=0.005, double time_to_live=0, double r=0., double g=1., double b=.8);
    void show_sampled_state(ompl::base::State * state);
    void show_sampled_robot_state(ompl::base::State * state);
    void show_edge(ompl::base::State*, ompl::base::State*, double edge_size=0.008);

    template<class T>
    void show_edge(const T t) {
        show_edge(t->state, t->parent->state);
    }

    void show_solution(std::vector<ompl::base::State *> states, double edge_size=0.05);

    template<class T>
    void show_solution(const T t) {
        show_solution(t->getStates());
    }

    void clear_markers();

};

#endif //ROS_MASTER_MOVEIT_OMPL_VISUALISER_H
