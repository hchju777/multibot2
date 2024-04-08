#include "multibot2_server/subgoal_generator/subgoalgen_config.h"

namespace multibot2_server::SubgoalGenerator
{
    void Config::declareParameters(const nav2_util::LifecycleNode::SharedPtr _nh)
    {
        _nh->declare_parameter("server.subgoal_generator.timeout", timeout_);

        _nh->declare_parameter("server.subgoal_generator.buffered_voronoi_diagram.min_offset", buffered_voronoi_diagram_.min_offset_);

        _nh->declare_parameter("server.subgoal_generator.velocity_obstacle.timeHorizon", velocity_obstacle_.timeHorizon_);

        _nh->declare_parameter("server.subgoal_generator.pibt.min_stopping_dist", pibt_.min_stopping_dist_);
        _nh->declare_parameter("server.subgoal_generator.pibt.min_truncated_area", pibt_.min_truncated_area_);
    }

    void Config::loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr _nh)
    {
        _nh->get_parameter_or("server.mode", mode_, mode_);
        _nh->get_parameter_or("server.subgoal_generator.timeout", timeout_, timeout_);

        _nh->get_parameter_or("server.subgoal_generator.buffered_voronoi_diagram.min_offset", buffered_voronoi_diagram_.min_offset_, buffered_voronoi_diagram_.min_offset_);

        _nh->get_parameter_or("server.subgoal_generator.velocity_obstacle.timeHorizon", velocity_obstacle_.timeHorizon_, velocity_obstacle_.timeHorizon_);

        _nh->get_parameter_or("server.subgoal_generator.pibt.min_stopping_dist", pibt_.min_stopping_dist_, pibt_.min_stopping_dist_);
        _nh->get_parameter_or("server.subgoal_generator.pibt.min_truncated_area", pibt_.min_truncated_area_, pibt_.min_truncated_area_);
    }
} // namespace multibot2_server::SubgoalGenerator