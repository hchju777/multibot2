#pragma once

#include <list>
#include <utility>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "multibot2_server/robot.h"
#include "multibot2_server/subgoal_generator/buffered_voronoi_diagram.h"

namespace multibot2_server::SubgoalGenerator::PIBT
{
    class CandidatesUtil
    {
    public:
        typedef BufferedVoronoiDiagram::VoronoiCell VoronoiCell;
        typedef BufferedVoronoiDiagram::VoronoiCellNoMap VoronoiCellNoMap;

    public:
        typedef std::pair<Point_2, CGAL::Polygon_with_holes_2<Kernel>> RawCandidate;
        typedef std::pair<std::string, std::list<CGAL::Polygon_with_holes_2<Kernel>>> Candidate;

    public:
        static std::list<RawCandidate> createRawCandidates(
            const Robot &_robot, const VoronoiCellNoMap &_vc_no_map, const VoronoiCell &_bvc, const VD &_vd);

        static std::list<CGAL::Polygon_with_holes_2<Kernel>> get_truncated_polygon(
            const CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes, const std::vector<Robot::Cone> &_cones);

    protected:
        static void create_triangular_subpolygons(
            const Point_2 &_site, const CGAL::Polygon_2<Kernel> &_vc_no_map, const VD &_vd,
            std::list<VoronoiCellNoMap> &_triangular_subpolygons);

        static void sort_triangular_subpolygons(std::list<VoronoiCellNoMap> &_triangular_subpolygons, const Robot &_robot);

        static void intersect_with_bvc(
            const std::list<VoronoiCellNoMap> &_triangular_subpolygons, const VoronoiCell &_bvc,
            std::list<RawCandidate> &_raw_candidates);

    }; // class CandidatesUtil
} // namespace multibot2_server::SubgoalGenerator::PIBT