#include "multibot2_server/subgoal_generator/velocity_obstacle.h"

namespace multibot2_server::SubgoalGenerator
{
    bool VelocityObstacle::updateVOCones(std::string _name)
    {
        if (not(robots_.contains(_name)))
        {
            std::cerr << "VelocityObstacle::updateVOCones: "
                      << "There is no robot named " << _name << std::endl;

            return false;
        }

        Robot& robot = robots_[_name];

        for (const auto &neighborPair : robot.neighbors())
        {
            const Robot other = neighborPair.second;

            const Eigen::Vector2d relativePosition = other.pose().position() - robot.pose().position();
            const Eigen::Vector2d relativeVelocity = robot.velocity() - other.velocity();

            const double distSq = relativePosition.squaredNorm();
            const double combinedRadius = robot.radius() + other.radius();
            const double combinedRadiusSq = combinedRadius * combinedRadius;

            Robot::Cone VOCone;
            VOCone.neighbor_ = other.name();
            VOCone.point_ = robot.pose().position() + other.velocity() * timeHorizon_;

            if (distSq > combinedRadiusSq)
            {
                // No collision
                const double leg = std::sqrt(distSq - combinedRadiusSq);
                VOCone.radius_ = leg;
                VOCone.left_direction_ =
                    Eigen::Vector2d(
                        relativePosition.x() * leg - relativePosition.y() * combinedRadius,
                        relativePosition.x() * combinedRadius + relativePosition.y() * leg) /
                    distSq;
                VOCone.right_direction_ =
                    Eigen::Vector2d(
                        relativePosition.x() * leg + relativePosition.y() * combinedRadius,
                        -relativePosition.x() * combinedRadius + relativePosition.y() * leg) /
                    distSq;
            }
            else
            {
                // Collision
                const Eigen::Vector2d w = relativeVelocity * timeHorizon_ - relativePosition;
                const double wLength = w.norm();
                const Eigen::Vector2d unitW = w / wLength;

                VOCone.radius_ = std::numeric_limits<double>::infinity();
                VOCone.left_direction_ = Eigen::Vector2d(unitW.y(), unitW.x());
                VOCone.right_direction_ = Eigen::Vector2d(unitW.y(), -unitW.x());
            }

            robot.VOCones().push_back(VOCone);
        }

        return true;
    }

    void VelocityObstacle::clearRobots()
    {
        Robots empty_robots;
        robots_.swap(empty_robots);
    }
} // namespace multibot2_server::SubgoalGenerator