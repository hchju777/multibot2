#include "multibot2_server/subgoal_generator/subgoal_generator.h"

namespace multibot2_server::SubgoalGenerator
{
    Generator::Generator(const Generator &_generator)
    {
        cfg_ = _generator.cfg_;

        robots_ = _generator.robots_;

        dynamic_graph_ = _generator.dynamic_graph_;

        solvers_ = _generator.solvers_;

        map_poly_ = _generator.map_poly_;
    }

    Generator &Generator::operator=(const Generator &_rhs)
    {
        if (&_rhs != this)
        {
            cfg_ = _rhs.cfg_;

            robots_ = _rhs.robots_;

            dynamic_graph_ = _rhs.dynamic_graph_;

            solvers_ = _rhs.solvers_;

            map_poly_ = _rhs.map_poly_;
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

    void Generator::update_map_polygon(nav2_costmap_2d::Costmap2D *_global_costmap,
                                       const costmap_converter::PolygonContainerConstPtr &_static_obstacles)
    {
        nav2_costmap_2d::Costmap2D *global_costmap = _global_costmap;

        double width = global_costmap->getSizeInMetersX();
        double height = global_costmap->getSizeInMetersY();

        std::vector<Point_2> box_map_vertices = {
            Point_2(global_costmap->getOriginX(), global_costmap->getOriginY()),
            Point_2(global_costmap->getOriginX() + width, global_costmap->getOriginY()),
            Point_2(global_costmap->getOriginX() + width, global_costmap->getOriginY() + height),
            Point_2(global_costmap->getOriginX(), global_costmap->getOriginY() + height)};

        map_poly_.outer_boundary() = CGAL::Polygon_2<Kernel>(box_map_vertices.begin(), box_map_vertices.end());

        auto static_obstacles = _static_obstacles;
        for (auto obst_iter = static_obstacles->begin(); obst_iter != static_obstacles->end(); ++obst_iter)
        {
            if (obst_iter->points.size() < 3)
                continue;

            std::vector<Eigen::Vector2d> vertices;
            for (const auto &vertex : obst_iter->points)
                vertices.push_back(Eigen::Vector2d(vertex.x, vertex.y));

            if (vertices.front().isApprox(vertices.back()))
                vertices.pop_back();

            CGAL::Polygon_2<Kernel> obst_poly;
            for (const auto &vertex : vertices)
                obst_poly.push_back(Point_2(vertex.x(), vertex.y()));

            std::list<CGAL::Polygon_with_holes_2<Kernel>> cropped_map_poly;
            CGAL::difference(map_poly_, obst_poly, std::back_inserter(cropped_map_poly));

            double max_area = 0.0;
            for (const auto &poly_w_holes : cropped_map_poly)
            {
                double area = CGAL::to_double(poly_w_holes.outer_boundary().area());
                for (const auto &hole : poly_w_holes.holes())
                    area -= CGAL::to_double(hole.area());

                if (area > max_area)
                {
                    max_area = area;
                    map_poly_ = poly_w_holes;
                }
            }
        }
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

            // for (const auto &vertexPair : group)
            // {
            //     std::cout << "\t" << vertexPair.first;
            // }
            // std::cout << std::endl;

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

            PIBT::Solver::SharedPtr solver = std::make_shared<PIBT::Solver>(cfg_, robotGroup, priority_graph, map_poly_);

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

            // Todo: Parameterize
            if (duration_sec.count() > 1.0)
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
            for (const auto &robotPair : singleThread.get())
            {
                std::string robotName = robotPair.first;
                Robot robot = robotPair.second;

                robots_[robotName].subgoal() = robot.subgoal();
                robots_[robotName].higher_neighbors() = robot.higher_neighbors();
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
        // for (auto &robot : robots_)
        //     robot.second.subgoal() = robot.second.pose();

        dynamic_graph_->addVertices(_robots);
    }
}