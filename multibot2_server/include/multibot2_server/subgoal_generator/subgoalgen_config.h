#pragma once

#include <memory>

#include <nav2_util/lifecycle_node.hpp>

namespace multibot2_server::SubgoalGenerator
{
    class Config
    {
    public:
        typedef std::unique_ptr<Config> UniquePtr;
        typedef std::shared_ptr<Config> SharedPtr;

    public:
        struct BufferedVoronoiDiagram
        {
            double min_offset_{2.7};
        } buffered_voronoi_diagram_; // struct BufferedVoronoiDiagram

        struct VelocityObstacle
        {
            double timeHorizon_{0.05};
        } velocity_obstacle_; // struct VelocityObstacle

        struct PIBT
        {
            double min_stopping_dist_{0.25};
            double min_truncated_area_{0.2};
        } pibt_; // struct PIBT

    public:
        Config() {}

    public:
        void declareParameters(const nav2_util::LifecycleNode::SharedPtr _nh);

        void loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr _nh);

        std::mutex &configMutex() { return config_mutex_; }

    private:
        std::mutex config_mutex_; //!< Mutex for config accesses and changes

    }; // class Config

} // namespace multibot2_server::SubgoalGenerator