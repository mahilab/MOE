#include <MOE/MOE.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Daq.hpp>
#include <Mahi/Robo.hpp>
// #include <Mahi/Gui.hpp>
#include <Mahi/Casadi/ModelControl.hpp>
#include <vector>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;
using namespace mahi::com;
using namespace moe;

using mahi::robo::WayPoint;

enum state {
    to_neutral_0,     // 0
    wrist_circle,     // 1
};

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

void to_state(state& current_state_, const state next_state_, WayPoint current_position_, WayPoint new_position_, Time traj_length_, MinimumJerk& mj_, Clock& ref_traj_clock_) {
    current_position_.set_time(seconds(0));
    new_position_.set_time(traj_length_);
    mj_.set_endpoints(current_position_, new_position_);
    
    if (!mj_.trajectory().validate()) {
        LOG(Warning) << "Minimum Jerk trajectory invalid.";
        stop = true;
    }
    current_state_ = next_state_;
    ref_traj_clock_.restart();
}

// returns vector of position and velocity of moe
std::vector<double> get_state(std::shared_ptr<Moe> moe_ptr){
    std::vector<double> state;
    for (size_t i = 0; i < moe_ptr->n_j; i++){
        state.push_back(moe_ptr->get_joint_position(i));
    }
    for (size_t i = 0; i < moe_ptr->n_j; i++){
        state.push_back(moe_ptr->get_joint_velocity(i));
    }
    // std::cout << state << std::endl;
    return state;
}

std::vector<double> get_traj(double curr_time, int n_dof, int ns, double step_size, std::vector<double> amps, std::vector<double> freqs, std::vector<double> offsets){
    std::vector<double> traj;
    for (auto i = 0; i < ns; i++){
        for (auto j = 0; j < n_dof * 2; j++){
            // if it is one of the position states
            if (j < n_dof) traj.push_back( 1 * amps[j%n_dof] * DEG2RAD * cos(2.0*PI*freqs[j%n_dof]*curr_time) + offsets[j%n_dof] - amps[j%n_dof]*DEG2RAD);
            // if it is one of the velocity states
            else           traj.push_back( -1 * amps[j%n_dof] * DEG2RAD * 2.0 * PI * freqs[j%n_dof]*sin(2.0*PI*freqs[j%n_dof]*curr_time));
            // std::cout << traj.back();
        }
        curr_time += step_size;
    }
    
    return traj;
}

int main(int argc, char* argv[]) {

    // make options
    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("c,calibrate", "Calibrates the MAHI Exo-II")
        ("n,no_torque", "trajectories are generated, but not torque provided")
        ("v,virtual", "example is virtual and will communicate with the unity sim")
        ("l,linear", "example uses linear mpc model of moe instead of nonlinear")
        ("p,pd_control", "example uses pd control instead of mpc")
        ("q, q_vec", "Q vector for MPC control", mahi::util::value<std::vector<double>>())
        ("r, r_vec", "R vector for MPC control", mahi::util::value<std::vector<double>>())
		("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    // if -h, print the help option
    if (result.count("help") > 0) {
        print_var(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

    /////////////////////////////////
    // construct and config MOE   //
    /////////////////////////////////

    std::shared_ptr<Moe> moe = nullptr;
    std::shared_ptr<Q8Usb> daq = nullptr;

    Time Ts = milliseconds(1);  // sample period for DAQ
    
    if(result.count("virtual") > 0){
        MoeConfigurationVirtual config_vr; 
        moe = std::make_shared<MahiOpenExoVirtual>(config_vr);
    }
    else{
        daq = std::make_shared<Q8Usb>();
        daq->open();

        MoeConfigurationHardware config_hw(*daq,VelocityEstimator::Hardware); 

        std::vector<TTL> idle_values(8,TTL_LOW);
        daq->DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
        daq->DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
        daq->DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);   

        moe = std::make_shared<MahiOpenExoHardware>(config_hw);
    }

    ////////////////////////////////

    //////////////////////////////////////////////
    // create MahiOpenExo and bind daq channels to it
    //////////////////////////////////////////////

    bool rps_is_init = false;

    //////////////////////////////////////////////

    // calibrate - manually zero the encoders (right arm supinated)
    if (result.count("calibrate") > 0) {
        moe->calibrate_auto(stop);
        LOG(Info) << "MAHI Exo-II encoders calibrated.";
        return 0;
    }

    // mahi::gui::PlotHelper plot_helper;
    // mahi::gui::run_plotter(plot_helper);

    // make MelShares
    MelShare ms_pos("ms_pos");
    MelShare ms_vel("ms_vel");
    MelShare ms_trq("ms_trq");
    MelShare ms_ref("ms_ref");

    // create ranges for saturating trajectories for safety  MIN            MAX
    std::vector<std::vector<double>> setpoint_rad_ranges = {{-90 * DEG2RAD, 30 * DEG2RAD},
                                                            {-90 * DEG2RAD, 90 * DEG2RAD},
                                                            {-60 * DEG2RAD, 60 * DEG2RAD},
                                                            {-60 * DEG2RAD, 60 * DEG2RAD}};

                                     // state 0    // state 1    // state 2    // state 3    // state 4    // state 5    // state 6
    std::vector<Time> state_times = {seconds(2.0), seconds(15.0)};

    casadi::Dict solver_opts;
    
                        //      0     1     2     3   
    std::vector<double> Q = {  10,   25,   20,   60,  // pos
                             0.10, 0.05, 0.05, 0.05}; // vel
    std::vector<double> R = {   3,   20,   40,   40}; // del_U
    std::vector<double> Rm ={  0,   0,   0,   0}; // magnitude
    std::vector<double> P = {  0,   0,   0,   0}; // integral

    std::vector<double> mpc_integral = {0,0,0,0};

    if (result.count("q_vec")) Q = result["q_vec"].as<std::vector<double>>();
    if (result.count("r_vec")) R = result["r_vec"].as<std::vector<double>>();

    std::cout << "before " << std::endl;

    ModelControl model_control(result.count("linear") ? "linear_moe" : "moe", Q, R, Rm, P);

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // setup trajectories

    double t = 0;

    Time mj_Ts = milliseconds(50);

    std::vector<double> ref;

    double freq_factor = 0.5;

	std::vector<double> sin_amplitudes = {30.0, 30.0, 30.0, 30.0};
	std::vector<double> sin_frequencies = {0.26*freq_factor, 0.36*freq_factor, 0.57*freq_factor, 0.82*freq_factor};

    // waypoints  
    WayPoint neutral_point = WayPoint(Time::Zero, {(-35+sin_amplitudes[0]) * DEG2RAD,   // Elbow F/E
                                                   ( 00+sin_amplitudes[1]) * DEG2RAD,   // Forearm P/S
                                                   ( 00+sin_amplitudes[2])  * DEG2RAD,  // Wrist F/E
                                                   (-10+sin_amplitudes[3]) * DEG2RAD}); // wrist R/U

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);
    timer.set_acceptable_miss_rate(0.05);

    // construct clock for regulating keypress
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(0.5);

    std::vector<std::string> dof_str = {"ElbowFE", "WristPS", "WristFE", "WristRU"};

    ////////////////////////////////////////////////
    //////////// State Manager Setup ///////////////
    ////////////////////////////////////////////////

    state current_state = to_neutral_0;
    WayPoint current_position;
    WayPoint new_position;
    Time traj_length;
    WayPoint dummy_waypoint = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});
    MinimumJerk mj(mj_Ts, dummy_waypoint, neutral_point.set_time(state_times[to_neutral_0]));
    std::vector<double> traj_max_diff = { 50 * DEG2RAD, 50 * DEG2RAD, 35 * DEG2RAD, 35 * DEG2RAD};
	mj.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);
    Clock ref_traj_clock;

    std::vector<double> aj_positions(4,0.0);
    std::vector<double> aj_velocities(4,0.0);

    std::vector<double> command_torques(4,0.0);

    ref_traj_clock.restart();
	
	// enable DAQ and exo
	moe->daq_enable();
	
    moe->enable();
	


    std::vector<double> traj = get_traj(0, moe->n_j, model_control.model_parameters.num_shooting_nodes, 
                                        model_control.model_parameters.step_size.as_seconds(), sin_amplitudes, sin_frequencies, neutral_point.get_pos());
    // std::cout << traj << std::endl;

    std::vector<std::vector<double>> data;
    std::vector<double> data_line;

    // trajectory following
    LOG(Info) << "Starting Movement.";

    //initialize kinematics
    moe->daq_read_all();
    moe->update();

    auto initial_mpc_state = neutral_point.get_pos();
    for (auto i = 0; i < moe->n_j; i++) initial_mpc_state.push_back(0);
    std::vector<double> initial_mpc_control = {0,0,0,0};
    
    model_control.set_state(mahi::util::seconds(0), initial_mpc_state, initial_mpc_control, traj, mpc_integral);
    model_control.start_calc();

    WayPoint start_pos(Time::Zero, moe->get_joint_positions());

    mj.set_endpoints(start_pos, neutral_point.set_time(state_times[to_neutral_0]));

    while (!stop) {
        // update all DAQ input channels
        moe->daq_read_all();

        // update MahiOpenExo kinematics
        moe->update();

        if (current_state != wrist_circle) {
            // update reference from trajectory
            ref = mj.trajectory().at_time(ref_traj_clock.get_elapsed_time());

            traj = get_traj(0.0, moe->n_j, model_control.model_parameters.num_shooting_nodes, 
                            model_control.model_parameters.step_size.as_seconds(), sin_amplitudes, sin_frequencies, neutral_point.get_pos());
            model_control.set_state(mahi::util::seconds(0), get_state(moe), command_torques, traj, mpc_integral);
        } 
        else {
            traj = get_traj(ref_traj_clock.get_elapsed_time().as_seconds(), moe->n_j, model_control.model_parameters.num_shooting_nodes, 
                            model_control.model_parameters.step_size.as_seconds(), sin_amplitudes, sin_frequencies, neutral_point.get_pos());
            static bool first_time = true;
            // if (first_time) std::cout << traj << std::endl;
            first_time = false;
            model_control.set_state(ref_traj_clock.get_elapsed_time(), get_state(moe), command_torques, traj, mpc_integral);
        }

        // constrain trajectory to be within range
        for (std::size_t i = 0; i < moe->n_j; ++i) {
            ref[i] = clamp(ref[i], setpoint_rad_ranges[i][0], setpoint_rad_ranges[i][1]);
        }
        
        // calculate anatomical command torques
        if (result.count("no_torque") > 0){
            command_torques = {0.0, 0.0, 0.0, 0.0};
            moe->set_raw_joint_torques(command_torques);
        }
        else{
            if (current_state != wrist_circle){
                command_torques = moe->set_pos_ctrl_torques(ref);
            } 
            else{
                if (!result.count("pd_control")){
                    command_torques = model_control.control_at_time(ref_traj_clock.get_elapsed_time()).u;
                } 
                else{
                    for (auto i = 0; i < 4; i++) ref[i] = traj[i];
                    command_torques = moe->set_pos_ctrl_torques(ref);
                }
            }
            // command_torques[3] *= 0;
            // command_torques[3] *= 0;
        }
        if (current_state == wrist_circle) moe->set_raw_joint_torques(command_torques);

        // if enough time has passed, continue to the next state. See to_state function at top of file for details
        if (ref_traj_clock.get_elapsed_time() > state_times[current_state]) {

            switch (current_state) {
                case to_neutral_0:
                    to_state(current_state, wrist_circle, neutral_point, neutral_point, state_times[wrist_circle], mj, ref_traj_clock);
                    break;
                case wrist_circle:
                    stop = true;
                    break;
            }
        }

        data_line.clear();
        // time
        data_line.push_back(t);
        // joint positions
        for (const auto &i : moe->get_joint_positions()) data_line.push_back(i);
        // joint velocities
        for (const auto &i : moe->get_joint_velocities()) data_line.push_back(i);
        // position ref
        if (current_state != wrist_circle) for (auto &&i : ref) data_line.push_back(i);
        else for (auto i = 0; i < 4; i++) data_line.push_back(traj[i]);
        // velocity ref
        if (current_state != wrist_circle) for (auto &&i : ref) data_line.push_back(0);
        else for (auto i = 4; i < 8; i++) data_line.push_back(traj[i]);
        // command torques
        for (auto &&i : command_torques) data_line.push_back(i);
        data.push_back(data_line);

        if (moe->any_limit_exceeded()) {
            stop = true;
        }

        // update all DAQ output channels
        if (!stop) moe->daq_write_all();

        // plot_helper.add_data("EFE Ref", traj[0]);
        // plot_helper.add_data("EFE Pos", moe->get_joint_position(0));
        // plot_helper.add_data("EFE Torque", moe->get_joint_command_torque(0));
        // plot_helper.write_data();

        // ms_ref.write_data(moe->get_joint_positions());
        // ms_pos.write_data(moe->get_joint_velocities());

        // std::cout << ref_traj_clock.get_elapsed_time().as_seconds() << std::endl << ref << std::endl << std::endl;

        // wait for remainder of sample period
        t = timer.wait().as_seconds();
    }
    command_torques = {0.0, 0.0, 0.0, 0.0, 0.0};
    moe->set_raw_joint_torques(command_torques);
    moe->daq_write_all();

    model_control.stop_calc();
    
    moe->daq_disable();
    moe->disable();

    disable_realtime();

    std::vector<std::string> header = {"Time (s)", 
                                         "EFE act (rad)",   "FPS act (rad)",   "WFE act (rad)",   "WRU act (rad)",
                                       "EFE act (rad/s)", "FPS act (rad/s)", "WFE act (rad/s)", "WRU act (rad/s)",
                                         "EFE ref (rad)",   "FPS ref (rad)",   "WFE ref (rad)",   "WRU ref (rad)",
                                       "EFE ref (rad/s)", "FPS ref (rad/s)", "WFE ref (rad/s)", "WRU ref (rad/s)",
                                          "EFE trq (Nm)",    "FPS trq (Nm)",    "WFE trq (Nm)",    "WRU trq (Nm)"};

    csv_write_row("data/rom_demo_results.csv",header);
    csv_append_rows("data/rom_demo_results.csv",data);

    // clear console buffer
    while (get_key_nb() != 0);

    return 0;
}
