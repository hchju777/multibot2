#include "multibot2_server/subgoal_generator/buffered_voronoi_diagram.h"

namespace multibot2_server::SubgoalGenerator
{
    BufferedVoronoiDiagram::BufferedVoronoiDiagram(const Config::SharedPtr &_cfg, const std::vector<Site_2> &_points)
        : cfg_(_cfg)
    {
        Kernel::Iso_rectangle_2 bbox = CGAL::bounding_box(_points.begin(), _points.end());

        auto offset_x = CGAL::abs(bbox.xmax() - bbox.xmin()) / 2;
        auto offset_y = CGAL::abs(bbox.ymax() - bbox.ymin()) / 2;

        auto offset = std::max(offset_x, offset_y);
        offset = std::max(offset, CGAL::Lazy_exact_nt<boost::multiprecision::mpq_rational>(cfg_->buffered_voronoi_diagram_.min_offset_));

        box_poly_.push_back(Point_2(bbox.xmin() - offset, bbox.ymin() - offset));
        box_poly_.push_back(Point_2(bbox.xmax() + offset, bbox.ymin() - offset));
        box_poly_.push_back(Point_2(bbox.xmax() + offset, bbox.ymax() + offset));
        box_poly_.push_back(Point_2(bbox.xmin() - offset, bbox.ymax() + offset));

        for (const auto &point : _points)
            vd_.insert(point);
    }

    bool BufferedVoronoiDiagram::get_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel> &_poly)
    {
        CGAL::Polygon_2<Kernel> vn_poly;
        if (not(get_raw_voronoi_polygon(_point, vn_poly)))
            return false;

        std::list<CGAL::Polygon_with_holes_2<Kernel>> cropped_vn_poly;
        CGAL::intersection(vn_poly, box_poly_, std::back_insert_iterator(cropped_vn_poly));

        if (cropped_vn_poly.empty())
            return false;

        CGAL::Polygon_with_holes_2<Kernel> &poly_w_holes = cropped_vn_poly.front();
        _poly = poly_w_holes.outer_boundary();

        return true;
    }

    bool BufferedVoronoiDiagram::get_polygon(
        const Point_2 &_point, CGAL::Polygon_with_holes_2<Kernel> &_poly,
        const std::vector<CGAL::Polygon_2<Kernel>> &_obstacles, double _offset)
    {
        CGAL::Polygon_2<Kernel> vn_poly;
        if (not(get_polygon(_point, vn_poly)))
            return false;

        CGAL::Lazy_exact_nt<boost::multiprecision::mpq_rational> offset(_offset);
        auto inner_offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(offset, vn_poly);

        if (inner_offset_polygons.empty())
            return false;

        std::list<CGAL::Polygon_with_holes_2<Kernel>> obstacles_union;
        CGAL::join(_obstacles.begin(), _obstacles.end(), std::back_inserter(obstacles_union));

        for (const auto &obst : obstacles_union)
        {
            std::list<CGAL::Polygon_with_holes_2<Kernel>> cropped_poly;
            CGAL::difference(_poly, obst.outer_boundary(), std::back_inserter(cropped_poly));

            double max_area = 0.0;
            for (const auto &poly_w_holes : cropped_poly)
            {
                double area = CGAL::to_double(poly_w_holes.outer_boundary().area());
                for (const auto &hole : poly_w_holes.holes())
                    area -= CGAL::to_double(hole.area());

                if (area > max_area)
                {
                    max_area = area;
                    _poly = poly_w_holes;
                }
            }
        }

        return true;
    }

    bool BufferedVoronoiDiagram::get_raw_voronoi_polygon(const Point_2 &_point, CGAL::Polygon_2<Kernel> &_poly)
    {
        assert(is_valid());

        Locate_result lr = vd_.locate(_point);

        if (Face_handle *f = boost::get<Face_handle>(&lr))
        {
            Ccb_halfedge_circulator ec_start = (*f)->ccb();
            Ccb_halfedge_circulator ec = ec_start;

            do
            {
                const CGAL::Object cur_seg_dual = vd_.dual().dual(ec->dual());

                const auto cur_seg = convert_to_seg(cur_seg_dual, ec->has_target(), _point);
                _poly.push_back(cur_seg.source());

                if (not(ec->has_target()))
                {
                    const CGAL::Object next_seg_dual = vd_.dual().dual(ec->next()->dual());
                    const auto next_seg = convert_to_seg(next_seg_dual, ec->next()->has_target(), _point);

                    _poly.push_back(next_seg.target());
                }
            } while (++ec != ec_start);

            if (_poly.size() == 2)
            {
                auto p1 = _poly.container().at(0);
                auto p2 = _poly.container().at(1);

                auto leg_vec = _point - p1;
                auto seg_vec = p2 - p1;

                Point_2 leg_point = p1 + CGAL::scalar_product(leg_vec, seg_vec) / CGAL::scalar_product(seg_vec, seg_vec) * seg_vec;
                auto normal_vec = _point - leg_point;

                _poly.push_back(Point_2(p2.x() + 1000 * normal_vec.direction().dx(),
                                        p2.y() + 1000 * normal_vec.direction().dy()));
                _poly.push_back(Point_2(p1.x() + 1000 * normal_vec.direction().dx(),
                                        p1.y() + 1000 * normal_vec.direction().dy()));
            }

            if (_poly.is_clockwise_oriented())
                _poly.reverse_orientation();

            return true;
        }

        return false;
    }

    Kernel::Segment_2 BufferedVoronoiDiagram::convert_to_seg(const CGAL::Object _seg_obj, bool _outgoing, const Point_2 &_point)
    {
        const Kernel::Segment_2 *dseg = CGAL::object_cast<Kernel::Segment_2>(&_seg_obj);
        const Kernel::Ray_2 *dray = CGAL::object_cast<Kernel::Ray_2>(&_seg_obj);
        const Kernel::Line_2 *dline = CGAL::object_cast<Kernel::Line_2>(&_seg_obj);

        if (dseg)
        {
            return *dseg;
        }
        else if (dray)
        {
            const auto &source = dray->source();
            const auto dsx = source.x();
            const auto dsy = source.y();
            const auto &dir = dray->direction();
            const auto tpoint = Kernel::Point_2(
                dsx + 1000 * dir.dx(), dsy + 1000 * dir.dy());

            if (_outgoing)
            {
                return Kernel::Segment_2(dray->source(), tpoint);
            }
            else
            {
                return Kernel::Segment_2(tpoint, dray->source());
            }
        }
        else
        {
            double a = CGAL::to_double(dline->a());
            double b = CGAL::to_double(dline->b());
            double c = CGAL::to_double(dline->c());

            const auto point = Kernel::Point_2(
                (-a * c + b * b * _point.x() - a * b * _point.y()) / std::sqrt(a * a + b * b),
                (-b * c - a * b * _point.x() + a * a * _point.y()) / std::sqrt(a * a + b * b));

            const auto dpx = point.x();
            const auto dpy = point.y();
            const auto &dir = dline->direction();
            const auto spoint = Kernel::Point_2(
                dpx - 1000 * dir.dx(), dpy - 1000 * dir.dy());
            const auto tpoint = Kernel::Point_2(
                dpx + 1000 * dir.dx(), dpy + 1000 * dir.dy());

            return Kernel::Segment_2(spoint, tpoint);
        }
    }

    bool BufferedVoronoiDiagram::check_point_in_poly_w_holes(
        const Point_2 &_point, const std::list<CGAL::Polygon_with_holes_2<Kernel>> &_poly_w_holes_list,
        CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes)
    {
        bool founded = false;

        for (const auto &candidate : _poly_w_holes_list)
        {
            auto os = CGAL::oriented_side(_point, candidate);

            if ((os == CGAL::ON_ORIENTED_BOUNDARY) or
                (os != CGAL::ON_ORIENTED_BOUNDARY and os == CGAL::POSITIVE))
            {
                _poly_w_holes = candidate;
                founded = true;
                break;
            }
        }

        return founded;
    }

    bool BufferedVoronoiDiagram::check_point_in_poly_w_holes(
        const Point_2 &_point, const std::vector<boost::shared_ptr<CGAL::Polygon_with_holes_2<Kernel>>> &_poly_w_holes_vec,
        CGAL::Polygon_with_holes_2<Kernel> &_poly_w_holes)
    {
        std::list<CGAL::Polygon_with_holes_2<Kernel>> poly_w_holes_list;
        poly_w_holes_list.clear();

        for (const auto &poly_w_holes : _poly_w_holes_vec)
            poly_w_holes_list.emplace_back(*poly_w_holes);

        return check_point_in_poly_w_holes(_point, poly_w_holes_list, _poly_w_holes);
    }
} // namespace multibot2_server::SubgoalGenerator