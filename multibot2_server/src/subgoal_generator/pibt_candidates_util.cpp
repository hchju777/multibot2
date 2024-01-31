#include "multibot2_server/subgoal_generator/pibt_candidates_util.h"

namespace multibot2_server::SubgoalGenerator::PIBT
{
    std::list<CandidatesUtil::RawCandidate> CandidatesUtil::createRawCandidates(
        const Robot &_robot, const VoronoiCellNoMap &_vc_no_map, const VoronoiCell &_bvc, const VD &_vd)
    {
        const Point_2 &site = _vc_no_map.first;
        const CGAL::Polygon_2<Kernel> &vc_no_map = _vc_no_map.second;

        std::list<VoronoiCellNoMap> triangular_subpolygons;
        create_triangular_subpolygons(site, vc_no_map, _vd, triangular_subpolygons);

        sort_triangular_subpolygons(triangular_subpolygons, _robot);

        std::list<RawCandidate> raw_candidates;
        intersect_with_bvc(triangular_subpolygons, _bvc, raw_candidates);

        return raw_candidates;
    }

    std::list<CGAL::Polygon_with_holes_2<Kernel>> CandidatesUtil::get_truncated_polygon(
        const CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes, const std::vector<Robot::Cone> &_cones)
    {
        std::list<CGAL::Polygon_with_holes_2<Kernel>> poly_w_holes_list;
        poly_w_holes_list.push_back(_poly_w_holes);

        for (const auto &cone : _cones)
        {
            CGAL::Polygon_2<Kernel> cone_polygon;
            cone_polygon.push_back(Point_2(cone.point_.x(), cone.point_.y()));

            double right_angle = std::atan2(cone.right_direction_.y(), cone.right_direction_.x());
            double left_angle = std::atan2(cone.left_direction_.y(), cone.left_direction_.x());
            left_angle = right_angle < left_angle ? left_angle : left_angle + 2 * M_PI;

            for (double angle = right_angle - 1e-8; angle < left_angle; angle = angle + M_PI / 18)
            {
                cone_polygon.push_back(Point_2(cone.point_.x() + 1000 * std::cos(angle),
                                               cone.point_.y() + 1000 * std::sin(angle)));
            }
            cone_polygon.push_back(Point_2(cone.point_.x() + 1000 * std::cos(left_angle + 1e-8),
                                           cone.point_.y() + 1000 * std::sin(left_angle + 1e-8)));

            std::list<CGAL::Polygon_with_holes_2<Kernel>> new_poly_w_holes_list;
            for (const auto &poly_w_holes : poly_w_holes_list)
            {
                std::list<CGAL::Polygon_with_holes_2<Kernel>> truncated_poly_w_holes;
                CGAL::difference(poly_w_holes, cone_polygon, std::back_inserter(truncated_poly_w_holes));

                for (const auto &poly_w_holes : truncated_poly_w_holes)
                    new_poly_w_holes_list.push_back(poly_w_holes);
            }
            poly_w_holes_list = new_poly_w_holes_list;
        }

        return poly_w_holes_list;
    }

    void CandidatesUtil::create_triangular_subpolygons(
        const Point_2 &_site, const CGAL::Polygon_2<Kernel> &_vc_no_map, const VD &_vd,
        std::list<VoronoiCellNoMap> &_triangular_subpolygons)
    {
        _triangular_subpolygons.clear();

        for (std::size_t i = 0; i < _vc_no_map.size(); ++i)
        {
            const Point_2 &p1 = _vc_no_map[i];
            const Point_2 &p2 = _vc_no_map[(i + 1) % _vc_no_map.size()];

            const Eigen::Vector3d v1(CGAL::to_double(_site.x() - p1.x()), CGAL::to_double(_site.y() - p1.y()), 0.0);
            const Eigen::Vector3d v2(CGAL::to_double(p2.x() - p1.x()), CGAL::to_double(p2.y() - p1.y()), 0.0);
            const double distance_between_edge_and_site = std::fabs(v1.cross(v2).z()) / v2.norm();

            const double sign = _vc_no_map.is_clockwise_oriented() ? 1.0 : -1.0;
            const double dx = sign * CGAL::to_double(p2.x() - p1.x());
            const double dy = sign * CGAL::to_double(p2.y() - p1.y());
            const Eigen::Vector2d unitOrthogonal = Eigen::Vector2d(dx, dy).unitOrthogonal();

            const Eigen::Vector2d vec_self_to_neighbor = 2 * distance_between_edge_and_site * unitOrthogonal;
            const Point_2 estimated_neighbor_site(_site.x() + vec_self_to_neighbor.x(), _site.y() + vec_self_to_neighbor.y());

            Locate_result neighbor_lr = _vd.locate(estimated_neighbor_site);
            Face_handle *neighbor_fh = boost::get<Face_handle>(&neighbor_lr);

            Point_2 neighbor_site;
            if (neighbor_fh)
                neighbor_site = (*neighbor_fh)->dual()->point();
            else
                continue;

            if (std::pow(CGAL::to_double(neighbor_site.x() - estimated_neighbor_site.x()), 2) +
                    std::pow(CGAL::to_double(neighbor_site.y() - estimated_neighbor_site.y()), 2) >
                1e-1)
            {
                neighbor_site = _site;
            }

            CGAL::Polygon_2<Kernel> triangular_subpolygon;

            triangular_subpolygon.push_back(_site);
            triangular_subpolygon.push_back(p1);
            triangular_subpolygon.push_back(p2);

            _triangular_subpolygons.push_back(std::make_pair(neighbor_site, triangular_subpolygon));
        }
    }

    void CandidatesUtil::sort_triangular_subpolygons(std::list<VoronoiCellNoMap> &_triangular_subpolygons, const Robot &_robot)
    {
        _triangular_subpolygons.sort(
            [_robot](const VoronoiCellNoMap &_c1, const VoronoiCellNoMap &_c2)
            {
                const Eigen::Vector3d goal_vec(_robot.goal().x() - _robot.pose().x(), _robot.goal().y() - _robot.pose().y(), 0);

                const std::vector<VoronoiCellNoMap> rawCandidateGroup = {_c1, _c2};
                std::vector<double> closest_leg_dots;

                for (size_t i = 0; i < rawCandidateGroup.size(); ++i)
                {
                    std::list<Eigen::Vector3d> legs;
                    for (const auto &p : rawCandidateGroup[i].second.container())
                    {
                        if (std::fabs(CGAL::to_double(p.x()) - _robot.pose().x()) < 1e-8 and
                            std::fabs(CGAL::to_double(p.y()) - _robot.pose().y()) < 1e-8)
                        {
                            continue;
                        }

                        legs.push_back(
                            Eigen::Vector3d(CGAL::to_double(p.x()) - _robot.pose().x(), CGAL::to_double(p.y()) - _robot.pose().y(), 0));
                    }

                    if (legs.front().cross(goal_vec).z() * legs.front().cross(legs.back()).z() > 0 and
                        legs.front().cross(goal_vec).z() * legs.back().cross(goal_vec).z() < 0)
                        return i == 0;

                    closest_leg_dots.push_back(std::max(
                        legs.front().dot(goal_vec) / legs.front().norm(),
                        legs.back().dot(goal_vec) / legs.back().norm()));
                }

                return closest_leg_dots.front() > closest_leg_dots.back();
            });
    }

    void CandidatesUtil::intersect_with_bvc(
        const std::list<VoronoiCellNoMap> &_triangular_subpolygons, const VoronoiCell &_bvc,
        std::list<RawCandidate> &_raw_candidates)
    {
        _raw_candidates.clear();

        for (const auto &triangular_subpolygon : _triangular_subpolygons)
        {
            std::list<CGAL::Polygon_with_holes_2<Kernel>> cropped_vn_poly;
            CGAL::intersection(triangular_subpolygon.second, _bvc.second, std::back_insert_iterator(cropped_vn_poly));

            CGAL::Polygon_with_holes_2<Kernel> triangular_bvc;

            bool founded = false;
            for (const auto &candidate : cropped_vn_poly)
            {
                auto os = CGAL::oriented_side(_bvc.first, candidate);

                if ((os == CGAL::ON_ORIENTED_BOUNDARY) or
                    (os != CGAL::ON_ORIENTED_BOUNDARY and os == CGAL::POSITIVE))
                {
                    triangular_bvc = candidate;
                    founded = true;
                    break;
                }
            }

            if (not(founded))
                continue;

            auto area = triangular_bvc.outer_boundary().area();
            for (const auto &hole : triangular_bvc.holes())
                area -= hole.area();

            if (CGAL::to_double(area) < 0.2)
                continue;

            _raw_candidates.push_back(std::make_pair(triangular_subpolygon.first, triangular_bvc));
        }
    }

} // namespace multibot2_server::SubgoalGenerator::PIBT