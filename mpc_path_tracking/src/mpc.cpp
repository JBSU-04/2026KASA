#include <ros/ros.h>
#include <mpc_path_tracking.h>

void FG_eval::operator()(ADvector &fg, const ADvector &vars)
{
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.

    fg[0] = 0;

    // 가중치는 Solve() 진입 시 updateCostWeights() 가 한 번만 계산해서 넣어준다.
    // 이 함수는 IPOPT 가 매 반복마다 호출하므로 여기서 파라미터 서버를 조회하면
    // solve 시간이 수십 ms 단위로 늘어난다.

    fg[0] += delta_start_cost_weight *
             CppAD::pow(vars[delta_start] - previous_delta_cost_reference, 2);

    // Cost for CTE, psi error and velocity
    for (int t = 0; t < N; t++)
    {
        fg[0] += cte_cost_effective_weight * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
        fg[0] += epsi_cost_effective_weight * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
        
        if (t < N - 1) {
            fg[0] += v_weight * CppAD::pow(vars[v_start + t] - vars[v_target_start + t], 2);
        } else {
            // 마지막 스텝은 마지막 v_target을 그대로 사용
            fg[0] += v_weight * CppAD::pow(vars[v_start + t] - vars[v_target_start + (N - 2)], 2);
        }
    }

    // Costs for steering (delta) and acceleration (a)
    for (int t = 0; t < N - 1; t++)
    {
        fg[0] += delta_cost_effective_weight * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += v_target_cost_weight * CppAD::pow(vars[v_target_start + t] - ref_v, 2);
    }

    // Costs related to the change in steering and acceleration (makes the ride smoother)
    for (int t = 0; t < N - 2; t++)
    {
        fg[0] += delta_change_cost_effective_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += v_target_change_cost_weight * CppAD::pow(vars[v_target_start + t + 1] - vars[v_target_start + t], 2);
    }

    // Setup Model Constraints

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int t = 1; t < N; t++)
    {
        // State at time t + 1
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        // The state at time t.
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        // Actuator constraints at time t only
        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> v_target0 = vars[v_target_start + t - 1];

        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

        // Setting up the rest of the model constraints
        
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
        fg[1 + v_start + t] = v1 - (v0 + k_v * (v_target0 - v0) * dt);
        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 / Lf * delta0 * dt);
    }
}

// 구간(도심/고주로)별 비용 가중치를 solve 당 한 번만 계산한다.
// is_highway_mode 는 mpc_path_tracking.h 가 /mission/active_section 의
// speed_limit_exempt 또는 CSV mission_state==3 으로 세팅한다.
void MPC::updateCostWeights()
{
    nh.param<double>("cost_weight/v_start_weight", v_weight, 20.0);
    nh.param<double>("cost_weight/delta_start_cost_weight", delta_start_cost_weight, 30000.0);
    nh.param<double>("cost_weight/v_target_cost_weight", v_target_cost_weight, 300.0);
    nh.param<double>("cost_weight/v_target_change_cost_weight", v_target_change_cost_weight, 100.0);

    double urban_cte_start = 3500.0;
    double urban_cte_urban = 1500.0;
    double urban_epsi = 2400.0;
    double urban_delta_start = 3500.0;
    double urban_delta_high = 38000.0;
    double urban_delta_change_start = 450000.0;
    double urban_delta_change_urban = 650000.0;
    double urban_speed_start = 7.0;
    double urban_speed_full = 15.0;

    nh.param<double>("urban_control/cte_cost_start_weight", urban_cte_start, 3500.0);
    nh.param<double>("urban_control/cte_cost_urban_weight", urban_cte_urban, 1500.0);
    nh.param<double>("urban_control/epsi_cost_weight", urban_epsi, 2400.0);
    nh.param<double>("urban_control/delta_cost_start_weight", urban_delta_start, 3500.0);
    nh.param<double>("urban_control/delta_cost_high_speed_weight", urban_delta_high, 38000.0);
    nh.param<double>("urban_control/delta_cost_start_mps", urban_speed_start, 7.0);
    nh.param<double>("urban_control/delta_cost_full_mps", urban_speed_full, 15.0);
    nh.param<double>("urban_control/delta_change_cost_start_weight", urban_delta_change_start, 450000.0);
    nh.param<double>("urban_control/delta_change_cost_urban_weight", urban_delta_change_urban, 650000.0);

    double hwy_cte = 1500.0;
    double hwy_epsi = 4200.0;
    double hwy_delta = 48000.0;
    double hwy_delta_change = 900000.0;

    nh.param<double>("highway_control/cte_cost_weight", hwy_cte, 1500.0);
    nh.param<double>("highway_control/epsi_cost_weight", hwy_epsi, 4200.0);
    nh.param<double>("highway_control/delta_cost_weight", hwy_delta, 48000.0);
    nh.param<double>("highway_control/delta_change_cost_weight", hwy_delta_change, 900000.0);

    // Static-obstacle paths need a faster lateral response than the urban
    // high-speed straight profile. This branch is active only while the
    // avoidance node publishes a finite speed cap; normal urban/highway MPC
    // fitting remains byte-for-byte equivalent in behavior.
    if (is_avoidance_mode) {
        nh.param<double>("avoidance_control/cte_cost_weight",
                         cte_cost_effective_weight, 4000.0);
        nh.param<double>("avoidance_control/epsi_cost_weight",
                         epsi_cost_effective_weight, 4000.0);
        nh.param<double>("avoidance_control/delta_cost_weight",
                         delta_cost_effective_weight, 12000.0);
        nh.param<double>("avoidance_control/delta_change_cost_weight",
                         delta_change_cost_effective_weight, 350000.0);
        return;
    }

    if (is_highway_mode) {
        cte_cost_effective_weight = hwy_cte;
        epsi_cost_effective_weight = hwy_epsi;
        delta_cost_effective_weight = hwy_delta;
        delta_change_cost_effective_weight = hwy_delta_change;
        return;
    }

    const double eval_speed = std::max(0.0, actual_speed_mps);
    if (eval_speed <= urban_speed_start) {
        cte_cost_effective_weight = urban_cte_start;
        epsi_cost_effective_weight = urban_epsi;
        delta_cost_effective_weight = urban_delta_start;
        delta_change_cost_effective_weight = urban_delta_change_start;
    } else if (eval_speed >= urban_speed_full) {
        cte_cost_effective_weight = urban_cte_urban;
        epsi_cost_effective_weight = urban_epsi;
        delta_cost_effective_weight = urban_delta_high;
        delta_change_cost_effective_weight = urban_delta_change_urban;
    } else {
        const double span = std::max(1.0e-3, urban_speed_full - urban_speed_start);
        double ratio = (eval_speed - urban_speed_start) / span;
        ratio = std::max(0.0, std::min(1.0, ratio));
        const double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);

        cte_cost_effective_weight = urban_cte_start + smooth_ratio * (urban_cte_urban - urban_cte_start);
        epsi_cost_effective_weight = urban_epsi;
        delta_cost_effective_weight = urban_delta_start + smooth_ratio * (urban_delta_high - urban_delta_start);
        delta_change_cost_effective_weight = urban_delta_change_start + smooth_ratio * (urban_delta_change_urban - urban_delta_change_start);
    }
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    size_t n_vars = state.size() * N + (N-1) * 2;
    size_t n_constraints = N * state.size();

    Dvector vars(n_vars);

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // TODO: Set lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) 
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // For delta. [-40, 40] in radians
    for (int i = delta_start; i < v_target_start; i++) 
    {
        vars_lowerbound[i] = steer_lowerBound;
        vars_upperbound[i] = steer_upperBound;
    }

    for (int i = v_target_start; i < n_vars; i++ ) {
        vars_lowerbound[i] = speed_lowerBound;
        vars_upperbound[i] = speed_upperBound;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++) 
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Start lower and upper limits at current values
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    updateCostWeights();
    // 조향 비용 배율: 모델 Lf 를 바꾸면 같은 회전에 필요한 조향이 Lf 에 비례해 커지므로,
    // 기존 튜닝의 공격성을 유지하려면 조향 항(delta_start/delta/delta_change)을 (Lf_old/Lf_new)^2 로 줄인다.
    {
        double steer_scale = 1.0;
        nh.param<double>("cost_weight/steer_scale", steer_scale, 1.0);
        steer_scale = std::max(0.05, std::min(steer_scale, 5.0));
        // delta_change(조향 변화율)만 따로 줄일 수 있게 한다. 기본은 steer_scale 과 같다.
        double change_scale = steer_scale;
        nh.param<double>("cost_weight/steer_change_scale", change_scale, steer_scale);
        change_scale = std::max(0.05, std::min(change_scale, 5.0));
        this->delta_start_cost_weight *= steer_scale;
        this->delta_cost_effective_weight *= steer_scale;
        this->delta_change_cost_effective_weight *= change_scale;
    }

    // 직선 감쇠(기본 꺼짐, scale 1.0): 앞으로 preview_s 동안 경로가 거의 직선이면 조향·조향변화 비용을 키운다.
    // 속도가 아니라 경로 곡률로 판단해, 앞에 곡선이 보이면(평균 곡률 >= kappa_zero) 1.0 이라 곡선 진입은 그대로다.
    {
        double sd_scale = 1.0, k_full = 0.004, k_zero = 0.012, sd_preview = 2.5;
        nh.param<double>("straight_damping/scale", sd_scale, 1.0);
        if (std::fabs(sd_scale - 1.0) > 1e-9 && !is_highway_mode && !is_avoidance_mode) {
            nh.param<double>("straight_damping/kappa_full", k_full, 0.004);
            nh.param<double>("straight_damping/kappa_zero", k_zero, 0.012);
            nh.param<double>("straight_damping/preview_s", sd_preview, 2.5);
            sd_scale = std::max(0.2, std::min(sd_scale, 5.0));
            const double v = std::max(1.0, static_cast<double>(state[3]));
            double ksum = 0.0;
            const int n = 12;
            for (int i = 0; i <= n; ++i) {
                const double xs = v * sd_preview * i / n;
                const double d1 = coeffs[1] + 2.0 * coeffs[2] * xs + 3.0 * coeffs[3] * xs * xs;
                const double d2 = 2.0 * coeffs[2] + 6.0 * coeffs[3] * xs;
                ksum += std::fabs(d2) / std::pow(1.0 + d1 * d1, 1.5);
            }
            const double kmean = ksum / (n + 1);
            const double straightness = std::max(0.0, std::min(1.0, (k_zero - kmean) / std::max(1e-6, k_zero - k_full)));
            const double f = 1.0 + straightness * (sd_scale - 1.0);
            this->delta_cost_effective_weight *= f;
            this->delta_change_cost_effective_weight *= f;
        }
    }

    fg_eval.ref_v = this->ref_v;
    fg_eval.dt    = this->dt;
    fg_eval.k_v   = this->k_v;
    fg_eval.Lf    = this->Lf;
    fg_eval.previous_delta_cost_reference = this->previous_delta_cost_reference;
    fg_eval.is_highway_mode = this->is_highway_mode;
    fg_eval.actual_speed_mps = this->actual_speed_mps;
    fg_eval.delta_cost_schedule_speed_mps = this->actual_speed_mps;

    fg_eval.v_weight = this->v_weight;
    fg_eval.delta_start_cost_weight = this->delta_start_cost_weight;
    fg_eval.v_target_cost_weight = this->v_target_cost_weight;
    fg_eval.v_target_change_cost_weight = this->v_target_change_cost_weight;
    fg_eval.cte_cost_effective_weight = this->cte_cost_effective_weight;
    fg_eval.epsi_cost_effective_weight = this->epsi_cost_effective_weight;
    fg_eval.delta_cost_effective_weight = this->delta_cost_effective_weight;
    fg_eval.delta_change_cost_effective_weight = this->delta_change_cost_effective_weight;

    // options for IPOPT solver
    string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    cost = solution.obj_value;
    this->cte_cost_effective_weight = fg_eval.cte_cost_effective_weight;
    this->epsi_cost_effective_weight = fg_eval.epsi_cost_effective_weight;
    this->delta_cost_effective_weight = fg_eval.delta_cost_effective_weight;
    this->delta_change_cost_effective_weight = fg_eval.delta_change_cost_effective_weight;
    
    // Returns the first actuator values, along with predicted x and y values to plot in the simulator
    vector<double> result;
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[v_target_start]);

    for (int i = 0; i < N; ++i)
    {
        result.push_back(solution.x[x_start + i]);
        result.push_back(solution.x[y_start + i]);
    }

    return result;
}

Eigen::VectorXd MPC::predict_FutureState(float velocity_actual, float delta, float cte, float epsi, double target_velocity, double dt)
{

    pred_px = velocity_actual * dt;
    pred_py = 0;
    pred_psi = velocity_actual * (-delta) * dt / Lf;
    pred_v = velocity_actual + k_v * (target_velocity - velocity_actual) * dt;
    pred_cte = cte + velocity_actual * sin(epsi) * dt;
    pred_epsi = epsi + pred_psi;

    Eigen::VectorXd state(6);
    state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

    return state;
}
