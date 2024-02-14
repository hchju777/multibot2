#pragma once

// standard includes
#include <memory>
#include <limits>
#include <cassert>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// CGAL includes
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Voronoi_diagram_2.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Polyline_simplification_2/simplify.h>

#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Polygon_offset_builder_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

#include <CGAL/convex_hull_2.h>

#include <CGAL/Triangulation_utils_2.h>
#include <CGAL/Voronoi_diagram_2/Face.h>
#include <CGAL/Voronoi_diagram_2/Handle_adaptor.h>
#include <CGAL/Voronoi_diagram_2/Vertex.h>
#include <CGAL/Voronoi_diagram_2/basic.h>
#include <CGAL/Voronoi_diagram_2/Accessor.h>

// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel InExact_Kernel;

typedef CGAL::Delaunay_triangulation_2<Kernel> DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT> AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP> VD;

// typedef for the result type of the point location
typedef AT::Site_2 Site_2;
typedef AT::Point_2 Point_2;
typedef VD::Locate_result Locate_result;
typedef VD::Vertex_handle Vertex_handle;
typedef VD::Face_handle Face_handle;
typedef VD::Halfedge_handle Halfedge_handle;
typedef VD::Ccb_halfedge_circulator Ccb_halfedge_circulator;

namespace multibot2_server::SubgoalGenerator
{
    class BufferedVoronoiDiagram
    {
    public:
        typedef std::unique_ptr<BufferedVoronoiDiagram> UniquePtr;
        typedef std::shared_ptr<BufferedVoronoiDiagram> SharedPtr;

    public:
        typedef std::pair<Point_2, CGAL::Polygon_with_holes_2<Kernel>> VoronoiCell;
        typedef std::pair<Point_2, CGAL::Polygon_2<Kernel>> VoronoiCellNoMap;

    public:
        BufferedVoronoiDiagram() {}

        BufferedVoronoiDiagram(const std::vector<Site_2> &_points, const CGAL::Polygon_with_holes_2<Kernel> &_map_poly);

        BufferedVoronoiDiagram(const BufferedVoronoiDiagram &_buffered_voronoi_diagram)
        {
            vd_ = _buffered_voronoi_diagram.vd_;
            box_poly_ = _buffered_voronoi_diagram.box_poly_;
            map_poly_ = _buffered_voronoi_diagram.map_poly_;
            min_offset_ = _buffered_voronoi_diagram.min_offset_;
            stop_ratio_ = _buffered_voronoi_diagram.stop_ratio_;
        }

        ~BufferedVoronoiDiagram() { vd_.clear(); }

    public:
        bool get_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel> &_poly);

        bool get_polygon(const Point_2 &_point, CGAL::Polygon_with_holes_2<Kernel> &_poly);

        bool get_raw_voronoi_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel> &_poly);

    public:
        bool convert_to_bvc(const Point_2 &_point, double _offset, CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes);

    public:
        BufferedVoronoiDiagram &operator=(const BufferedVoronoiDiagram &_rhs)
        {
            if (&_rhs != this)
            {
                vd_ = _rhs.vd_;
                box_poly_ = _rhs.box_poly_;
                map_poly_ = _rhs.map_poly_;
                min_offset_ = _rhs.min_offset_;
                stop_ratio_ = _rhs.stop_ratio_;
            }

            return *this;
        }

    public:
        inline VD &vd() { return vd_; }
        inline const VD &vd() const { return vd_; }

        inline void reset() { vd_.clear(); }

        inline bool is_valid() { return vd_.is_valid(); }

    protected:
        Kernel::Segment_2 convert_to_seg(const CGAL::Object _seg_obj, bool _outgoing);

        bool check_point_in_poly_w_holes(
            const Point_2 &_point, const std::list<CGAL::Polygon_with_holes_2<Kernel>> &_poly_w_holes_list,
            CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes);

        bool check_point_in_poly_w_holes(
            const Point_2 &_point, const std::vector<boost::shared_ptr<CGAL::Polygon_with_holes_2<Kernel>>> &_poly_w_holes_vec,
            CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes);

    protected:
        VD vd_;

        CGAL::Polygon_2<Kernel> box_poly_;

        CGAL::Polygon_with_holes_2<Kernel> map_poly_;

        double min_offset_{2.7};
        double stop_ratio_{0.1};

    }; // class BufferedVoronoiDiagram
} // namespace multibot2_server::SubgoalGenerator