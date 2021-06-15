#include "tools.h"
#include "logging.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

static double EPS = 0.00000001;

Tools::Tools() = default;

Tools::~Tools() = default;


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.empty()) {
        Logging::logging("Invalid estimation or ground_truth data", Logging::ERROR);
        return rmse;
    }
    // accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        // coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // calculate the mean
    rmse /= estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    MatrixXd Hj(3, 4);
    // recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // check division by zero
    if (px == 0 & py == 0) {
        Logging::logging("[CalculateJacobian] Division by 0", Logging::WARNING);
        return Hj;
    }
    double pL = pow(px, 2) + pow(py, 2);
    // compute the Jacobian matrix
    Hj << px / sqrt(pL), py / sqrt(pL), 0, 0,
            -1 * py / pL, px / pL, 0, 0,
            py * (vx * py - vy * px) / (pow(pL, 1.5)), px * (vy * px - vx * py) / (pow(pL, 1.5)), px / sqrt(pL), py /
                                                                                                                 sqrt(pL);
    return Hj;
}


VectorXd Tools::TransformPolar2Cart(const VectorXd &x_state) {
    double meas_rho = x_state[0];
    double meas_phi = x_state[1];
    //double meas_rho_dot = x_state[2];

    VectorXd x_cart = VectorXd(4);
    x_cart << meas_rho * cos(meas_phi),
            meas_rho * sin(meas_phi),
            0,
            0;
    return x_cart;
}


VectorXd Tools::TransformCart2Polar(const VectorXd &x_state) {
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // Division 0 check, set rho bec we can not skip this step
    double rho = sqrt(px * px + py * py);
    if (fabs(rho) < EPS){
        Logging::logging("[TransformCart2Polar] Division by 0", Logging::WARNING);
        rho = EPS;
    }

    VectorXd x_pol = VectorXd(3);
    x_pol << rho, // rho
            atan2(py, px), // phi
            (px * vx + py * vy) / rho; // rho dot
    return x_pol;
}


// More elegant modulo range version than while subtract / add pi
// From https://stackoverflow.com/a/4635752 Lior Kogan
// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
double Tools::Mod(double x, double y)
{
    static_assert(!std::numeric_limits<double>::is_exact , "Mod: floating-point type expected");
    if (fabs(y) < EPS)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
        }
    }

    return m;
}