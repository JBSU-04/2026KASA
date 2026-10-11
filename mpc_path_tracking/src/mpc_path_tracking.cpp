#include <mpc.h>

// Evaluate a polynomial.
double MPC::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;

    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }

    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPC::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) 
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i <= order; i++) 
        {
            A(j, i) = pow(xvals(j), i);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    //cout << "result is : " << result << endl;
    //cout << "---------------------" << endl;
    return result;
}

