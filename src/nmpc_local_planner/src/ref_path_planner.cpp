// Function to generate Bézier curve points
#include "nmpc_local_planner/ref_path_planner.h"
#include <boost/algorithm/clamp.hpp> 


namespace nmpc_local_planner {

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}    

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


double get_smooth_velocity(double &v_set, double &robot_vel, double &acc_lim, double &max_vel, double &delta_time)
{
    double acc_factor = acc_lim * delta_time;
    // double decel_factor = acc_lim * delta_time;
    double out_speed;
    
    if (robot_vel >= 0.0)
        out_speed = robot_vel + boost::algorithm::clamp(v_set - robot_vel, -acc_factor, acc_factor );
    else
        out_speed = robot_vel + boost::algorithm::clamp(v_set - robot_vel,- acc_factor, acc_factor );

    out_speed = boost::algorithm::clamp(out_speed,-max_vel,max_vel);
    return out_speed;

}


std::vector<BezierPoint> generateBezierCurve(const BezierPoint& P0, const BezierPoint& P1, const BezierPoint& P2, const BezierPoint& P3, int segments) {
    std::vector<BezierPoint> curvePoints;
    for (int i = 0; i <= segments; ++i) {
        double t = i / (double)segments;
        double oneMinusT = 1 - t;
        BezierPoint point = P0 * (oneMinusT * oneMinusT * oneMinusT) + 
                    P1 * (3 * oneMinusT * oneMinusT * t) + 
                    P2 * (3 * oneMinusT * t * t) + 
                    P3 * (t * t * t);
        curvePoints.push_back(point);
    }
    return curvePoints;
}

// Function to calculate the derivative of x with respect to t
double dx_dt(double t) {
    return 1;
}

// Function to calculate the derivative of y with respect to t
double dy_dt(double t, Eigen::VectorXd &coeffs) {
    return 3 * coeffs[3] * std::pow(t, 2) + 2 * coeffs[2] * std::pow(t, 1)+ coeffs[1] * std::pow(t, 0);
}

// Function to calculate the derivative of y with respect to t
double dy_dtdt(double t, Eigen::VectorXd &coeffs) {
    return 6 * coeffs[3] * t + 2 * coeffs[2];
}

double theta(double t, Eigen::VectorXd &coeffs) {
    return atan(3 * coeffs[3] * std::pow(t, 2) + 2 * coeffs[2] * std::pow(t, 1)+ coeffs[1] * std::pow(t, 0));
}

double kappa (double t, Eigen::VectorXd &coeffs) {
    return dy_dtdt(t,coeffs)/pow(1+pow(dy_dt(t,coeffs),2) ,1.5);
}


// Speed function v(t)
double v(double t, Eigen::VectorXd &coeffs) {
    return std::sqrt(std::pow(dx_dt(t), 2) + std::pow(dy_dt(t, coeffs), 2));
}

// Numerical integration using the trapezoidal rule
double integrate(std::function<double(double, Eigen::VectorXd&)> func, double a, double b, int n, Eigen::VectorXd &coeffs) {

    double h = (b - a) / n;
    double sum = 0.5 * (func(a,coeffs) + func(b,coeffs));
    for (int i = 1; i < n; ++i) {
        double x = a + i * h;
        sum += func(x,coeffs);
    }
    return sum * h;
}

// Bisection method to find t for a given arc length S
double find_t_for_given_S(double S_given, double t0, double t1,  Eigen::VectorXd &coeffs) {
    double a = t0;
    double b = t1;
    double c;
    int maxIter = 100;
        double tol = 1e-6;
    for (int i = 0; i < maxIter; ++i) {
        c = (a + b) / 2;
        double S_c = integrate(v, t0, c, 10, coeffs);

        if (std::abs(S_c - S_given) < tol || (b - a) / 2 < tol) {
            return c;
        }

        if ((S_c < S_given) ? true : false) {
            a = c;
        } else {
            b = c;
        }
    }

    return c; // Return the approximate solution
}


}
    
