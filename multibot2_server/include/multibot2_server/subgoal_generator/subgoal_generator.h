#pragma once

#include <memory>
#include <future>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include "costmap_converter/costmap_to_polygons.h"

#include "multibot2_server/robot.h"

#include "multibot2_server/subgoal_generator/subgoalgen_config.h"
#include "multibot2_server/subgoal_generator/dynamic_graph.h"
#include "multibot2_server/subgoal_generator/velocity_obstacle.h"
#include "multibot2_server/subgoal_generator/pibt.h"

namespace multibot2_server::SubgoalGenerator
{
    class Generator
    {
    public:
        typedef std::unique_ptr<Generator> UniquePtr;
        typedef std::shared_ptr<Generator> SharedPtr;

    public:
        Generator(const Config::SharedPtr &_cfg) : cfg_(_cfg) { reset(); }

        Generator(const Generator &_generator);

        ~Generator() { reset(); }

    public:
        Generator &operator=(const Generator &_rhs);

    public:
        void update_subgoals(Robots &_robots);

    protected:
        void generate_solvers();

        bool check_if_need_to_replan(const DynamicGraph::Vertices &_group);

        void find_subgoals();

    protected:
        void reset();

        void reset(const Robots &_robots);

    protected:
        Config::SharedPtr cfg_;

        Robots robots_;

        DynamicGraph::SharedPtr dynamic_graph_{std::make_shared<DynamicGraph>()};

        std::vector<PIBT::Solver::SharedPtr> solvers_;

    }; // class Generator
} // namespace multibot2_server::SubgoalGenerator