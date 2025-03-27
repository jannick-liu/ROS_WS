#ifndef PATH_H
#define PATH_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>


namespace nmpc_local_planner
{
class  Path
{
private:
    /* data */
public:
    Path() : /* dynamic_(false) */
    {}
    virtual ~Path()
    {}

    virtual double calculatePathLength(Eigen::VectorXd xvals, Eigen::VectorXd yvals) = 0;
    virtual Eigen::VectorXd curvefit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) = 0;


protected:
    // bool dynamic_;
    // Eigen::Vector2d centroid_velocity_;
};


class polyPath : public Path
{
    public:

    SegmentObstacle() : Obstacle(double x1, double y1, double x2, double y2) : Obstacle()
    {

    }
}

}