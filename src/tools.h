#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"
class Tools {
public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                         const std::vector<Eigen::VectorXd> &ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

    static Eigen::VectorXd TransformPolar2Cart(const Eigen::VectorXd &x_state);

    static Eigen::VectorXd TransformCart2Polar(const Eigen::VectorXd &x_state);

    static double Mod(double x, double y);
};

#endif  // TOOLS_H_
