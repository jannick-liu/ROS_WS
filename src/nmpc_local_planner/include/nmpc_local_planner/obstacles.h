#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>


namespace nmpc_local_planner
{
class  Obstacle
{
private:
    /* data */
public:
    Obstacle() : dynamic_(false)
    {}
    virtual ~Obstacle()
    {}

    virtual const Eigen::Vector2d& getCentroid() const = 0;
    virtual double getMinimumDistance(const Eigen::Vector2d& position) const = 0;
    virtual double getCloestPoint(const Eigen::Vector2d& position) const = 0;
    virtual Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


protected:
    bool dynamic_;
    Eigen::Vector2d centroid_velocity_;
};


class SegmentObstacle : public Obstacle
{
    public:

    SegmentObstacle(double x1, double y1, double x2, double y2) : Obstacle()
    {

    }
}

}