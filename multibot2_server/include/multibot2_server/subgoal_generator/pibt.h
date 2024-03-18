#pragma once

#include <memory>
#include <stack>

#include "multibot2_server/robot.h"
#include "multibot2_server/subgoal_generator/buffered_voronoi_diagram.h"

#include "multibot2_server/subgoal_generator/subgoalgen_config.h"
#include "multibot2_server/subgoal_generator/pibt_candidates_util.h"
#include "multibot2_server/subgoal_generator/pibt_subgoal_util.h"

namespace multibot2_server::SubgoalGenerator::PIBT
{
    class Solver
    {
    public:
        typedef std::unique_ptr<Solver> UniquePtr;
        typedef std::shared_ptr<Solver> SharedPtr;

    public:
        typedef CandidatesUtil::VoronoiCell VoronoiCell;
        typedef CandidatesUtil::VoronoiCellNoMap VoronoiCellNoMap;

        typedef CandidatesUtil::Candidate Candidate;

    public:
        Solver() {}

        Solver(const Config::SharedPtr &_cfg, const Robots &_robots, std::stack<std::string> _priority_graph, const CGAL::Polygon_with_holes_2<Kernel> &_map_poly);

        Solver(const Solver &_solver);

        ~Solver() {}

    public:
        Robots &robots() { return robots_; }
        const Robots &robots() const { return robots_; }

    public:
        Solver &operator=(const Solver &_rhs);

    public:
        Robots solve();

    protected:
        bool priorityInheritance(
            const std::string _robotName,
            std::set<std::string> &_close, std::set<std::string> &_open);

        bool priorityInheritance(
            const std::string _childName, const std::string _parentName,
            std::set<std::string> &_close, std::set<std::string> &_open);

        bool priorityInheritance(
            const std::string _robotName, std::list<Candidate> _candidates,
            std::set<std::string> &_close, std::set<std::string> &_open);

    protected:
        std::list<Candidate> createCandidates(
            Robot &_robot,
            const std::set<std::string> &_close, const std::set<std::string> &_open,
            std::string _parent = std::string());

    protected:
        bool set_bvc_generator(const CGAL::Polygon_with_holes_2<Kernel> &_map_poly);

        bool generate_diagrams();

    protected:
        bool validate_subgoal(std::string _robotName, const Point_2 &_subgoal);

        bool is_in_the_same_face(const Point_2 &_p1, const Point_2 &_p2);

    protected:
        Config::SharedPtr cfg_;

        Robots robots_;

        BufferedVoronoiDiagram::SharedPtr bvc_generator_;

        std::map<std::string, VoronoiCellNoMap> voronoi_diagram_w_no_map_;
        std::map<std::string, VoronoiCell> buffered_voronoi_diagram_;

        std::list<std::string> priority_list_;

    }; // class Solver
} // namespace multibot2_server::SubgoalGenerator::PIBT