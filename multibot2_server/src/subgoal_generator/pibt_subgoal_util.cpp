#include "multibot2_server/subgoal_generator/pibt_subgoal_util.h"

namespace multibot2_server::SubgoalGenerator::PIBT
{
    bool SubgoalUtil::find_subgoal(
        const Point_2 &_goal, const std::list<CGAL::Polygon_with_holes_2<Kernel>> &_subpolygons,
        Point_2 &_subgoal)
    {
        double min_objective_value = std::numeric_limits<double>::max();
        for (const auto &subpolygon : _subpolygons)
        {
            std::list<CGAL::Polygon_2<Kernel>> triangular_subpolygons = triangular_decompose(subpolygon);

            Point_2 local_subgoal;
            double local_objective_value;

            if (not(SubgoalUtil::find_subgoal(_goal, triangular_subpolygons, local_subgoal, local_objective_value)))
                continue;

            if (min_objective_value > local_objective_value)
            {
                min_objective_value = local_objective_value;
                _subgoal = local_subgoal;
            }
        }

        return (min_objective_value != std::numeric_limits<double>::max());
    }

    bool SubgoalUtil::find_subgoal(
        const Point_2 &_goal, const std::list<CGAL::Polygon_2<Kernel>> &_convex_subpolygons,
        Point_2 &_subgoal, double &_min_objective_value)
    {
        _min_objective_value = std::numeric_limits<double>::max();

        for (const auto &subpolygon : _convex_subpolygons)
        {
            Point_2 local_subgoal = _goal;

            if (subpolygon.bounded_side(_goal) == CGAL::ON_UNBOUNDED_SIDE)
            {
                std::min_element(subpolygon.edges_begin(), subpolygon.edges_end(),
                                 [_goal, &local_subgoal](const auto &_a, const auto &_b)
                                 {
                                     const std::list<CGAL::Segment_2<Kernel>> edgeList = {_a, _b};
                                     std::list<Point_2> subgoalList;

                                     for (const auto &edge : edgeList)
                                     {
                                         const CGAL::Vector_2<Kernel> edgeVec = edge.target() - edge.source();
                                         const CGAL::Vector_2<Kernel> legVec = _goal - edge.source();

                                         if (CGAL::scalar_product(edgeVec, legVec) < 0)
                                             subgoalList.push_back(edge.source());
                                         else if (CGAL::scalar_product(edgeVec, legVec) > CGAL::scalar_product(edgeVec, edgeVec))
                                             subgoalList.push_back(edge.target());
                                         else
                                             subgoalList.push_back(
                                                 edge.source() + CGAL::scalar_product(edgeVec, legVec) / CGAL::scalar_product(edgeVec, edgeVec) * edgeVec);
                                     }

                                     if (CGAL::squared_distance(_goal, subgoalList.front()) < CGAL::squared_distance(_goal, subgoalList.back()))
                                     {
                                         local_subgoal = subgoalList.front();
                                         return true;
                                     }
                                     else
                                     {
                                         local_subgoal = subgoalList.back();
                                         return false;
                                     }
                                 });
            }

            const double local_objective_value = CGAL::to_double(CGAL::squared_distance(local_subgoal, _goal));
            if (local_objective_value < _min_objective_value)
            {
                _min_objective_value = local_objective_value;
                _subgoal = local_subgoal;
            }
        }

        return (_min_objective_value != std::numeric_limits<double>::max());
    }

    std::list<CGAL::Polygon_2<Kernel>> SubgoalUtil::triangular_decompose(const CGAL::Polygon_with_holes_2<Kernel> &_cell_w_holes)
    {
        CGAL::Polygon_triangulation_decomposition_2<Kernel> triangular_decomp;

        std::list<CGAL::Polygon_2<Kernel>> triangular_decomp_poly_list;
        triangular_decomp(_cell_w_holes, std::back_inserter(triangular_decomp_poly_list));

        return triangular_decomp_poly_list;
    }
    
} // namespace multibot2_server::SubgoalGenerator::PIBT