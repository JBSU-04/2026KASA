#ifndef MPC_H
#define MPC_H
#include <ros/ros.h>
#include <array>
#include <Eigen/Core>
#include <Eigen/QR>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;
using CppAD::AD;
using Eigen::VectorXd;

class MPC {
public:
    ros::NodeHandle nh;
    int mpc_steps;
    size_t N;
    double dt;

    const double ref_cte = 0;
    const double ref_epsi = 0;
    double ref_v;
    double k_v;
    float steer_upperBound;
    float steer_lowerBound;
    double speed_upperBound;
    double speed_lowerBound;

    size_t x_start; 
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t v_target_start;

    double Lf;

    const double x0 = 0;
    const double y0 = 0;
    const double psi0 = 0;
    const double cte0 = 0;
    const double epsi0 = 0;

    double cte_cost_weight;
    double epsi_cost_weight;
    double cte_cost_effective_weight;
    double epsi_cost_effective_weight;
    double v_weight;
    double delta_cost_weight;
    bool speed_scheduled_delta_cost_enabled;
    double delta_cost_schedule_speed_mps;
    double delta_cost_start_mps;
    double delta_cost_full_mps;
    double delta_cost_start_weight;
    double delta_cost_high_speed_weight;
    double delta_cost_highway_mps;
    double delta_cost_highway_weight;
    double delta_cost_effective_weight;
    double delta_start_cost_weight;
    double v_target_cost_weight;
    double delta_change_cost_weight;
    double delta_change_cost_effective_weight;
    double v_target_change_cost_weight;
    double previous_delta_cost_reference;
    double cost;
    bool is_highway_mode{false};
    bool is_avoidance_mode{false};
    double actual_speed_mps{0.0};

    MPC() {
        // ROS 파라미터 읽기
        nh.param<int>("N", mpc_steps, 50);
        nh.param<double>("dt", dt, 0.1);
        nh.param<double>("k_v", k_v, 0.4);
        nh.param<double>("ref_v", ref_v, 20.0);
        nh.param<double>("Lf", Lf, 1.5);
        nh.param<float>("steer/upperBound", steer_upperBound, 0.6981);
        nh.param<float>("steer/lowerBound", steer_lowerBound, -0.6981);
        nh.param<double>("speed/upperBound", speed_upperBound, 20.0);
        nh.param<double>("speed/lowerBound", speed_lowerBound, 0);
        previous_delta_cost_reference = 0.0;
        speed_scheduled_delta_cost_enabled = false;
        delta_cost_schedule_speed_mps = 0.0;
        delta_cost_start_mps = 10.0;
        delta_cost_full_mps = 15.0;
        delta_cost_start_weight = 5000.0;
        delta_cost_high_speed_weight = 500000.0;
        delta_cost_highway_mps = 25.0;
        delta_cost_highway_weight = 55000.0;
        delta_cost_effective_weight = 0.0;


        // N 초기화
        N = (size_t)mpc_steps;

        // 의존하는 변수 초기화
        x_start = 0;
        y_start = x_start + N;
        psi_start = y_start + N;
        v_start = psi_start + N;
        cte_start = v_start + N;
        epsi_start = cte_start + N;
        delta_start = epsi_start + N;
        v_target_start = delta_start + N - 1;
    };


	virtual ~MPC(){};

	// Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    // 구간별 비용 가중치를 solve 당 한 번만 갱신한다(FG_eval 내부 조회 금지).
    void updateCostWeights();

	//Predicts Future State of the car according to Kinematic Model.
    Eigen::VectorXd predict_FutureState(float velocity_actual, float delta, float cte,float epsi, double target_velocity, double dt);
    double polyeval(Eigen::VectorXd coeffs, double x);
	Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    void broadcastTransform();
private:
    
    
    // Predicted state values
    double pred_px = 0;
    double pred_py = 0;
    double pred_psi=0;
    double pred_v = 0;
    double pred_cte = 0;
    double pred_epsi = 0;
    


};

class FG_eval : public MPC {
public:
    
	// Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

	FG_eval(Eigen::VectorXd coeffs) {
        this->coeffs = coeffs;
    }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	
	void operator()(ADvector& fg, const ADvector& vars);
	
};

#endif
