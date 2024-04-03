#include "multibot2_server/subgoal_generator/subgoal_generator.h"

namespace multibot2_server::SubgoalGenerator
{
    Generator::Generator(const Generator &_generator)
    {
        cfg_ = _generator.cfg_;

        robots_ = _generator.robots_;

        dynamic_graph_ = _generator.dynamic_graph_;

        solvers_ = _generator.solvers_;
    }

    Generator &Generator::operator=(const Generator &_rhs)
    {
        if (&_rhs != this)
        {
            cfg_ = _rhs.cfg_;

            robots_ = _rhs.robots_;

            dynamic_graph_ = _rhs.dynamic_graph_;

            solvers_ = _rhs.solvers_;
        }

        return *this;
    }

    void Generator::update_subgoals(Robots &_robots)
    {
        reset(_robots);

        generate_solvers();

        find_subgoals();

        _robots = robots_;
    }

    void Generator::generate_solvers()
    {
        solvers_.clear();

        std::stack<std::string> priority_graph = dynamic_graph_->topologicalSort();

        for (const auto &group : dynamic_graph_->generateGroupList())
        {
            if (group.empty())
                continue;

            if (group.size() == 1)
            {
                std::string robotName = group.begin()->first;

                robots_[robotName].front() = robotName;
                robots_[robotName].subgoal() = robots_[robotName].goal();

                continue;
            }

            if (not(check_if_need_to_replan(group)))
                continue;

            for (auto iter = group.begin(); iter != group.end(); ++iter)
            {
                Vertices::const_iterator front_it;

                if (iter != group.begin())
                    front_it = std::prev(iter, 1);
                else
                    front_it = std::prev(group.end(), 1);

                std::string robotName = iter->first;
                Robot &robot = robots_[robotName];

                robot.front() = front_it->first;
                robot.replan_time() = std::chrono::system_clock::now();
            }

            Robots robotGroup;
            VelocityObstacle::UniquePtr vo_generator = std::make_unique<VelocityObstacle>(cfg_);
            for (const auto &vertexPair : group)
            {
                std::string robotName = vertexPair.first;

                vo_generator->emplaceRobot(robots_[robotName]);
                robotGroup.emplace(robotName, robots_[robotName]);
            }

            for (const auto &robotPair : vo_generator->robots())
            {
                std::string robotName = robotPair.first;

                vo_generator->updateVOCones(robotName);
                robots_[robotName].VOCones() = vo_generator->robots()[robotName].VOCones();
            }

            PIBT::Solver::SharedPtr solver = std::make_shared<PIBT::Solver>(cfg_, robotGroup, priority_graph);

            solvers_.push_back(solver);
        }
    }

    bool Generator::check_if_need_to_replan(const DynamicGraph::Vertices &_group)
    {
        // Check group change
        for (auto iter = _group.begin(); iter != _group.end(); ++iter)
        {
            Vertices::const_iterator front_it;

            if (iter != _group.begin())
                front_it = std::prev(iter, 1);
            else
                front_it = std::prev(_group.end(), 1);

            std::string robotName = iter->first;

            if (robots_[robotName].front() != front_it->first)
                return true;
        }

        // Check Timeout
        for (const auto &vertexPair : _group)
        {
            const Robot &robot = robots_[vertexPair.first];

            std::chrono::duration<double> duration_sec = std::chrono::system_clock::now() - robot.replan_time();

            if (duration_sec.count() > cfg_->timeout_)
                return true;
        }

        return false;
    }

    void Generator::find_subgoals()
    {
        std::vector<std::future<Robots>> multiThread;
        multiThread.clear();

        for (const auto &solver : solvers_)
        {
            multiThread.push_back(
                std::async(std::launch::async, [this, solver]()
                           { return solver->solve(); }));
        }

        for (auto &singleThread : multiThread)
        {
            const multibot2_server::Robots &robots = singleThread.get();

            bool recovery = true;
            for (const auto &robotPair : robots)
            {
                const multibot2_server::Robot &robot = robotPair.second;
                
                double dist = (robot.subgoal().position() - robot.pose().position()).norm();
                if (dist > 1e-8)
                {
                    recovery = false;
                    break;
                }
            }

            for (const auto &robotPair : robots)
            {
                std::string robotName = robotPair.first;
                Robot robot = robotPair.second;

                robots_[robotName].higher_neighbors() = robot.higher_neighbors();

                if (not(recovery))
                    robots_[robotName].subgoal() = robot.subgoal();
                else
                    robots_[robotName].subgoal() = robot.goal();
            }
        }
    }

    void Generator::reset()
    {
        Robots empty_robots;
        robots_.swap(empty_robots);
        robots_.clear();

        dynamic_graph_->reset();

        std::vector<PIBT::Solver::SharedPtr> empty_solvers;
        solvers_.swap(empty_solvers);
        solvers_.clear();
    }

    void Generator::reset(const Robots &_robots)
    {
        reset();

        robots_ = _robots;

        dynamic_graph_->addVertices(_robots);
    }
}