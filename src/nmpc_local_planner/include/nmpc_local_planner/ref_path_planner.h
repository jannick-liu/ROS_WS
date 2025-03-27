#ifndef REF_PATH_PLANNER_H_
#define REF_PATH_PLANNER_H_
#endif

#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"



using namespace std;
namespace nmpc_local_planner
{
    struct BezierPoint {
    double x, y;

    // Constructor for convenience
    BezierPoint(double x, double y) : x(x), y(y) {}

    // Overloaded operators to make our calculations cleaner
    BezierPoint operator*(double scalar) const {
        return BezierPoint(x * scalar, y * scalar);
    }

    BezierPoint operator+(const BezierPoint& other) const {
        return BezierPoint(x + other.x, y + other.y);
    }
    };

    // class ref_path_planner
    // {
    // private:
    //     /* data */
    // public:
    //     ref_path_planner(/* args */);
    //     ~ref_path_planner();

    std::vector<BezierPoint> generateBezierCurve(const BezierPoint& P0, const BezierPoint& P1, const BezierPoint& P2, const BezierPoint& P3, int segments);
    double find_t_for_given_S(double S_given, double t0, double t1,  Eigen::VectorXd &coeffs);
    double integrate(std::function<double(double, Eigen::VectorXd&)> func, double a, double b, int n, Eigen::VectorXd &coeffs);
    // Function to calculate the derivative of x with respect to t
    double dx_dt(double t);

    // Function to calculate the derivative of y with respect to t
    double dy_dt(double t, Eigen::VectorXd &coeffs);

    double theta(double t, Eigen::VectorXd &coeffs);

    double kappa(double t, Eigen::VectorXd &coeffs);


    // Speed function v(t)
    double v(double t, Eigen::VectorXd &coeffs);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    double polyeval(Eigen::VectorXd coeffs, double x); 

    double get_smooth_velocity(double &v_set, double &robot_vel, double &acc_lim, double &max_vel, double &delta_time);


}