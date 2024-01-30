#include "multibot2_server/server_node.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    MultibotServer::MultibotServer()
        : nav2_util::LifecycleNode("server", "", true)
    {
        init_variables();

        std::chrono::duration<double> duration{instance_manager_->subgoal_generator_duration()};
        subgoal_update_timer_ = nh_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(duration), std::bind(&MultibotServer::generate_subgoals, this));

        RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
    }

    MultibotServer::~MultibotServer()
    {
        RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
    }

    void MultibotServer::run_server_panel(int argc, char *argv[])
    {
        QApplication app(argc, argv);

        serverPanel_ = std::make_unique<Panel>(nh_, instance_manager_);
        serverPanel_->attach(*this);
        serverPanel_->show();

        panel_is_running_ = true;

        app.exec();
    }

    void MultibotServer::update(const multibot2_util::PanelUtil::Msg &_msg)
    {
        if (not(panel_is_running_))
            return;

        switch (_msg)
        {
        case multibot2_util::PanelUtil::Request::PLAN_REQUEST:
        {
            break;
        }

        case multibot2_util::PanelUtil::Request::START_REQUEST:
        {
            break;
        }

        default:
            break;
        }
    }

    void MultibotServer::init_variables()
    {
        nh_ = std::shared_ptr<::nav2_util::LifecycleNode>(this, [](::nav2_util::LifecycleNode *) {});

        instance_manager_ = std::make_shared<Instance_Manager>(nh_);

        panel_is_running_ = false;
    }

    void MultibotServer::generate_subgoals()
    {
        SubgoalGenerator::DynamicGraph::UniquePtr dynamic_graph = std::make_unique<SubgoalGenerator::DynamicGraph>();

        std::map<std::string, Robot> robots;
        for (const auto &robot_rosPair : instance_manager_->robots())
        {
            const Robot &robot = robot_rosPair.second.robot_;

            robots.emplace(robot.name(), robot);
        }

        // Todo: Move to Subgoal Generator Constructor
        CGAL::Polygon_with_holes_2<Kernel> map_poly;
        {
            nav2_costmap_2d::Costmap2D *global_costmap = instance_manager_->global_costmap_ros()->getCostmap();
            double width = global_costmap->getSizeInMetersX();
            double height = global_costmap->getSizeInMetersY();

            std::vector<Point_2> box_map_vertices = {
                Point_2(global_costmap->getOriginX(), global_costmap->getOriginY()),
                Point_2(global_costmap->getOriginX() + width, global_costmap->getOriginY()),
                Point_2(global_costmap->getOriginX() + width, global_costmap->getOriginY() + height),
                Point_2(global_costmap->getOriginX(), global_costmap->getOriginY() + height)};

            map_poly.outer_boundary() = CGAL::Polygon_2<Kernel>(box_map_vertices.begin(), box_map_vertices.end());

            auto static_obstacles = instance_manager_->static_obstacles();
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
                CGAL::difference(map_poly, obst_poly, std::back_inserter(cropped_map_poly));

                double max_area = 0.0;
                for (const auto &poly_w_holes : cropped_map_poly)
                {
                    double area = CGAL::to_double(poly_w_holes.outer_boundary().area());
                    for (const auto &hole : poly_w_holes.holes())
                        area -= CGAL::to_double(hole.area());

                    if (area > max_area)
                    {
                        max_area = area;
                        map_poly = poly_w_holes;
                    }
                }
            }

            // std::cout << "- Polygon:" << std::endl;
            // for (const auto &p : map_poly.outer_boundary().container())
            // {
            //     std::cout << "  - vertex: [" << p.x() << "," << p.y() << "]" << std::endl;
            // }
            // for (const auto &hole : map_poly.holes())
            // {
            //     std::cout << "- Polygon:" << std::endl;
            //     for (const auto &p : hole.container())
            //     {
            //         std::cout << "  - vertex: [" << p.x() << "," << p.y() << "]" << std::endl;
            //     }
            // }
        }

        dynamic_graph->addVertices(robots);
    }
} // namespace multibot2_server