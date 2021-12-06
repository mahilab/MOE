#include <MOE/MOE.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Daq.hpp>
#include <Mahi/Robo.hpp>
#include <vector>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;
using namespace mahi::com;
using namespace moe;

using mahi::robo::WayPoint;

enum state {
    to_neutral_0,     // 0
    to_bottom_elbow,  // 1
    to_top_elbow,     // 2
    to_neutral_1,     // 3
    to_top_wrist,     // 4
    wrist_circle,     // 5
    to_neutral_2      // 6
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

int main(int argc, char* argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make options
    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("c,calibrate", "Calibrates the MAHI Exo-II")
        ("n,no_torque", "trajectories are generated, but not torque provided")
        ("v,virtual", "example is virtual and will communicate with the unity sim")
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
    
    if(result.count("virtual") > 0){
        MoeConfigurationVirtual config_vr; 
        moe = std::make_shared<MahiOpenExoVirtual>(config_vr);
    }
    else{
        daq = std::make_shared<Q8Usb>();
        daq->open();

        MoeConfigurationHardware config_hw(*daq,VelocityEstimator::Software); 

        std::vector<TTL> idle_values(8,TTL_LOW);
        daq->DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
        daq->DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
        daq->DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);   

        moe = std::make_shared<MahiOpenExoHardware>(config_hw);
    }

    Time Ts = milliseconds(1);  // sample period for DAQ

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

    // make MelShares
    MelShare ms_pos("ms_pos");
    MelShare ms_vel("ms_vel");
    MelShare ms_trq("ms_trq");
    MelShare ms_ref("ms_ref");

    // create ranges for saturating trajectories for safety  MIN            MAX
    std::vector<std::vector<double>> setpoint_rad_ranges = {{-90 * DEG2RAD, 15 * DEG2RAD},
                                                            {-90 * DEG2RAD, 90 * DEG2RAD},
                                                            {-60 * DEG2RAD, 60 * DEG2RAD},
                                                            {-60 * DEG2RAD, 60 * DEG2RAD}};

                                     // state 0    // state 1    // state 2    // state 3    // state 4    // state 5    // state 6
    std::vector<Time> state_times = {seconds(2.0), seconds(2.0), seconds(4.0), seconds(2.0), seconds(1.0), seconds(4.0), seconds(1.0)};

    // setup trajectories

    double t = 0;

    Time mj_Ts = milliseconds(50);

    std::vector<double> ref;

    // waypoints                                   Elbow F/E       Forearm P/S   Wrist F/E     Wrist R/U     LastDoF
    WayPoint neutral_point = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});
    WayPoint bottom_elbow  = WayPoint(Time::Zero, {-65 * DEG2RAD,  45 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});
    WayPoint top_elbow     = WayPoint(Time::Zero, {  5 * DEG2RAD, -45 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});
    WayPoint top_wrist     = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 25 * DEG2RAD});

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
    std::vector<double> traj_max_diff = { 60 * DEG2RAD, 60 * DEG2RAD, 100 * DEG2RAD, 60 * DEG2RAD};
	mj.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);
    Clock ref_traj_clock;

    std::vector<double> aj_positions(4,0.0);
    std::vector<double> aj_velocities(4,0.0);

    std::vector<double> command_torques(4,0.0);

    ref_traj_clock.restart();
	
	// enable DAQ and exo
	moe->daq_enable();
	
    moe->enable();
	
	// moe->daq_watchdog_start();    

    // trajectory following
    LOG(Info) << "Starting Movement.";

    std::vector<std::vector<double>> data;
    std::vector<double> data_line;

    //initialize kinematics
    moe->daq_read_all();
    moe->update();

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
        } 
        else {
            ref[0] = neutral_point.get_pos()[0];
            ref[1] = neutral_point.get_pos()[1];
            ref[2] = 45.0 * DEG2RAD * sin(2.0 * PI * ref_traj_clock.get_elapsed_time() / state_times[wrist_circle]);
            ref[3] = 30.0 * DEG2RAD * cos(2.0 * PI * ref_traj_clock.get_elapsed_time() / state_times[wrist_circle]) - 5.0 * DEG2RAD;
        }

        // constrain trajectory to be within range
        for (std::size_t i = 0; i < moe->n_j; ++i) {
            ref[i] = clamp(ref[i], setpoint_rad_ranges[i][0], setpoint_rad_ranges[i][1]);
        }
        
        // calculate anatomical command torques
        if (result.count("no_torque") > 0){
            command_torques = {0.0, 0.0, 0.0, 0.0, 0.0};
            moe->set_raw_joint_torques(command_torques);
        }
        else{
            command_torques = moe->set_pos_ctrl_torques(ref);
        }

        // if enough time has passed, continue to the next state. See to_state function at top of file for details
        if (ref_traj_clock.get_elapsed_time() > state_times[current_state]) {

            switch (current_state) {
                case to_neutral_0:
                    to_state(current_state, to_bottom_elbow, neutral_point, bottom_elbow, state_times[to_bottom_elbow], mj, ref_traj_clock);
                    break;
                case to_bottom_elbow:
                    to_state(current_state, to_top_elbow, bottom_elbow, top_elbow, state_times[to_top_elbow], mj, ref_traj_clock);
                    break;
                case to_top_elbow:
                    to_state(current_state, to_neutral_1, top_elbow, neutral_point, state_times[to_neutral_1], mj, ref_traj_clock);
                    break;
                case to_neutral_1:
                    to_state(current_state, to_top_wrist, neutral_point, top_wrist, state_times[to_top_wrist], mj, ref_traj_clock);
                    break;
                case to_top_wrist:
                    to_state(current_state, wrist_circle, top_wrist, top_wrist, state_times[wrist_circle], mj, ref_traj_clock);
                    break;
                case wrist_circle:
                    to_state(current_state, to_neutral_2, top_wrist, neutral_point, state_times[to_neutral_2], mj, ref_traj_clock);
                    break;
                case to_neutral_2:
                    stop = true;
                    break;
            }
        }

        data_line.clear();
        data_line.push_back(t);
        for (auto &&i : ref) data_line.push_back(i);
        for (auto &&i : moe->get_joint_positions()) data_line.push_back(i);
        for (auto &&i : moe->get_joint_velocities()) data_line.push_back(i);
        for (auto &&i : moe->get_joint_command_torques(0)) data_line.push_back(i);
        data.push_back(data_line);
        

        // kick watchdog
        // if (!moe->daq_watchdog_kick() || moe->any_limit_exceeded()) {
        //     stop = true;
        // }
        if (moe->any_limit_exceeded()) {
            stop = true;
        }

        // update all DAQ output channels
        if (!stop) moe->daq_write_all();

        ms_ref.write_data(moe->get_joint_positions());
        ms_pos.write_data(moe->get_joint_velocities());

        // wait for remainder of sample period
        t = timer.wait().as_seconds();
    }

    command_torques = {0.0, 0.0, 0.0, 0.0, 0.0};
    moe->set_raw_joint_torques(command_torques);
    moe->daq_write_all();


    std::vector<std::string> header = {"Time (s)", 
                                       "EFE ref (rad)", "FPS ref (rad)", "WFE ref (rad)", "WRU ref (rad)",
                                       "EFE act (rad)", "FPS act (rad)", "WFE act (rad)", "WRU act (rad)",
                                       "EFE act (rad/s)", "FPS act (rad/s)", "WFE act (rad/s)", "WRU act (rad/s)",
                                       "EFE trq (Nm)", "FPS trq (Nm)", "WFE trq (Nm)", "WRU trq (Nm)"};

    csv_write_row("data/rom_demo_results.csv",header);
    csv_append_rows("data/rom_demo_results.csv",data);
    
    moe->daq_disable();
    moe->disable();

    disable_realtime();

    // clear console buffer
    while (get_key_nb() != 0);

    return 0;
}