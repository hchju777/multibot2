#include "multibot2_server/subgoal_generator/pibt.h"

namespace multibot2_server::SubgoalGenerator::PIBT
{
    Solver::Solver(const Config::SharedPtr &_cfg, const Robots &_robots, std::stack<std::string> _priority_graph)
        : cfg_(_cfg), robots_(_robots)
    {
        priority_list_.clear();
        while (not(_priority_graph.empty()))
        {
            std::string robotName = _priority_graph.top();
            _priority_graph.pop();

            if (robots_.contains(robotName))
                priority_list_.push_back(robotName);
        }

        set_bvc_generator();

        if (not(generate_diagrams()))
        {
            std::cerr << "PIBT::Solver::Solver(): "
                      << "Fail to generate voronoi diagram." << std::endl;
        }
    }

    Solver::Solver(const Solver &_solver)
    {
        cfg_ = _solver.cfg_;

        robots_ = _solver.robots_;

        bvc_generator_ = _solver.bvc_generator_;

        voronoi_diagram_w_no_map_ = _solver.voronoi_diagram_w_no_map_;

        buffered_voronoi_diagram_ = _solver.buffered_voronoi_diagram_;

        priority_list_ = _solver.priority_list_;
    }

    Solver &Solver::operator=(const Solver &_rhs)
    {
        if (&_rhs != this)
        {
            cfg_ = _rhs.cfg_;

            robots_ = _rhs.robots_;

            bvc_generator_ = _rhs.bvc_generator_;

            voronoi_diagram_w_no_map_ = _rhs.voronoi_diagram_w_no_map_;

            buffered_voronoi_diagram_ = _rhs.buffered_voronoi_diagram_;

            priority_list_ = _rhs.priority_list_;
        }

        return *this;
    }

    Robots Solver::solve()
    {
        std::set<std::string> initial_open, open, close;
        for (const auto &robotPair : robots_)
        {
            std::string robotName = robotPair.first;

            initial_open.emplace(robotName);
            open.emplace(robotName);
        }

        while (not(open.empty()) and not(priority_list_.empty()))
        {
            Robot robot = robots_[priority_list_.front()];

            // priorityInheritance(robot.name(), close, open);
            if (not(priorityInheritance(robot.name(), close, open)))
            {
                open = initial_open;
                close.clear();

                priority_list_.pop_front();

                continue;
            }

            priority_list_.remove_if([&open](std::string _robotName)
                                     { return not(open.contains(_robotName)); });
        }

        return robots_;
    }

    bool Solver::priorityInheritance(
        const std::string _robotName,
        std::set<std::string> &_close, std::set<std::string> &_open)
    {
        if (not(_open.contains(_robotName)))
            return true;

        std::list<Candidate> candidates = createCandidates(
            robots_[_robotName], _close, _open);

        return priorityInheritance(_robotName, candidates, _close, _open);
    }

    bool Solver::priorityInheritance(
        const std::string _childName, const std::string _parentName,
        std::set<std::string> &_close, std::set<std::string> &_open)
    {
        if (not(_open.contains(_childName)))
            return true;

        std::list<Candidate> candidates = createCandidates(
            robots_[_childName], _close, _open, _parentName);

        return priorityInheritance(_childName, candidates, _close, _open);
    }

    bool Solver::set_bvc_generator()
    {
        std::vector<Site_2> points;

        for (const auto &robotPair : robots_)
        {
            const Robot &robot = robotPair.second;

            points.push_back(Site_2(robot.pose().x(), robot.pose().y()));
        }

        bvc_generator_ = std::make_shared<BufferedVoronoiDiagram>(cfg_, points);

        return true;
    }

    bool Solver::priorityInheritance(
        const std::string _robotName,
        std::list<Candidate> _candidates, std::set<std::string> &_close, std::set<std::string> &_open)
    {
        Robot &robot = robots_[_robotName];
        robot.higher_neighbors() = _close;

        _open.erase(_robotName);

        while (not(_candidates.empty()))
        {
            Candidate candidate = _candidates.front();
            std::string candidateName = candidate.first;

            if (cfg_->mode_ != "V-RVO")
            {
                _close.emplace(candidateName);

                if (_robotName != candidateName)
                {
                    if (not(priorityInheritance(candidateName, _robotName, _close, _open)))
                    {
                        _candidates.remove_if([&_close](Candidate _candidate)
                                              { return _close.contains(_candidate.first); });

                        continue;
                    }
                }
            }

            Point_2 goal(robot.goal().x(), robot.goal().y());
            Point_2 subgoal;

            if (SubgoalUtil::find_subgoal(goal, candidate.second, subgoal))
            {
                multibot2_util::Pose subgoal_pose(CGAL::to_double(subgoal.x()), CGAL::to_double(subgoal.y()), 0.0);
                double sqDist = (subgoal_pose - robot.pose()).position().norm();

                if (sqDist > cfg_->pibt_.min_stopping_dist_)
                {
                    robot.subgoal() = subgoal_pose;

                    return true;
                }
            }
            _candidates.pop_front();
        }

        robot.subgoal() = robot.pose();

        return false;
    }

    std::list<Solver::Candidate> Solver::createCandidates(
        Robot &_robot,
        const std::set<std::string> &_close, const std::set<std::string> &_open, std::string _parent)
    {
        std::set<std::string> close = _close;

        if (_parent != std::string())
            close.emplace(_parent);

        std::list<Candidate> candidates;

        std::vector<Robot::Cone> cones;

        for (auto iter = _robot.VOCones().begin(); iter != _robot.VOCones().end();)
        {
            if (_open.contains(iter->neighbor_))
            {
                iter = _robot.VOCones().erase(iter);
            }
            else
            {
                cones.push_back(*iter);
                ++iter;
            }
        }

        if (not(voronoi_diagram_w_no_map_.contains(_robot.name())) or not(buffered_voronoi_diagram_.contains(_robot.name())))
            return std::list<Solver::Candidate>();

        auto rawCandidates = CandidatesUtil::createRawCandidates(
            _robot,
            voronoi_diagram_w_no_map_[_robot.name()], buffered_voronoi_diagram_[_robot.name()],
            bvc_generator_->vd());
        for (const auto rawCandidatePair : rawCandidates)
        {
            const Point_2 &neighborSite = rawCandidatePair.first;

            std::pair<std::string, Robot> neighborPair = *std::min_element(
                robots_.begin(), robots_.end(),
                [neighborSite](const auto &_robotPair1, const auto &_robotPair2)
                {
                    const double dx1 = _robotPair1.second.pose().x() - CGAL::to_double(neighborSite.x());
                    const double dy1 = _robotPair1.second.pose().y() - CGAL::to_double(neighborSite.y());

                    const double dx2 = _robotPair2.second.pose().x() - CGAL::to_double(neighborSite.x());
                    const double dy2 = _robotPair2.second.pose().y() - CGAL::to_double(neighborSite.y());

                    return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
                });

            if (neighborPair.first != _robot.name() and close.contains(neighborPair.first))
                continue;

            const CGAL::Polygon_with_holes_2<Kernel> &triangular_subpolygon = rawCandidatePair.second;
            const std::list<CGAL::Polygon_with_holes_2<Kernel>> &truncated_polygon = CandidatesUtil::get_truncated_polygon(triangular_subpolygon, cones);

            double truncated_area = std::accumulate(truncated_polygon.begin(), truncated_polygon.end(), 0.0,
                                                    [](double _sum, const CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes)
                                                    {
                                                        _sum += CGAL::to_double(_poly_w_holes.outer_boundary().area());

                                                        for (const auto &hole : _poly_w_holes.holes())
                                                            _sum -= CGAL::to_double(hole.area());

                                                        return _sum;
                                                    });

            if (truncated_area < cfg_->pibt_.min_truncated_area_)
                continue;

            candidates.emplace_back(neighborPair.first, truncated_polygon);
        }

        return candidates;
    }

    bool Solver::generate_diagrams()
    {
        for (const auto &robotPair : robots_)
        {
            const Robot &robot = robotPair.second;

            const Point_2 site = Point_2(robot.pose().x(), robot.pose().y());

            VoronoiCellNoMap voronoi_cell_w_no_map;
            voronoi_cell_w_no_map.first = site;
            if (not(bvc_generator_->get_polygon(site, voronoi_cell_w_no_map.second)))
                continue;

            voronoi_diagram_w_no_map_.emplace(robot.name(), voronoi_cell_w_no_map);

            VoronoiCell buffered_voronoi_cell;
            buffered_voronoi_cell.first = site;

            // Todo: When update from foxy(CGAL 5.0) to humble(5.4), Somtimes process has died when creating Buffered Voronoi Diagram using CGAL. Temporarily inactivated
            // if (cfg_->mode_ == "V-PIBT")
            // {
            //     if (not(bvc_generator_->get_polygon(site, buffered_voronoi_cell.second, robot.local_obstacles(), robot.radius())))
            //         continue;
            // }
            // else if (cfg_->mode_ == "V-RVO")
                buffered_voronoi_cell.second.outer_boundary() = voronoi_cell_w_no_map.second;

            buffered_voronoi_diagram_.emplace(robot.name(), buffered_voronoi_cell);
        }

        return true;
    }

    bool Solver::validate_subgoal(std::string _robotName, const Point_2 &_subgoal)
    {
        if (not(robots_.contains(_robotName)))
        {
            std::cerr << "PIBT::Solver::validate_subgoal: "
                      << "There is no robot named " << _robotName << std::endl;
            return false;
        }

        const auto &robot = robots_[_robotName];

        return is_in_the_same_face(Point_2(robot.pose().x(), robot.pose().y()), _subgoal);
    }

    bool Solver::is_in_the_same_face(const Point_2 &_p1, const Point_2 &_p2)
    {
        Locate_result p1_lr = bvc_generator_->vd().locate(_p1);
        Locate_result p2_lr = bvc_generator_->vd().locate(_p2);

        Face_handle *p1_fh = boost::get<Face_handle>(&p1_lr);
        Face_handle *p2_fh = boost::get<Face_handle>(&p2_lr);

        return (p1_fh == p2_fh);
    }

} // namespace multibot2_server::SubgoalGenerator::PIBT