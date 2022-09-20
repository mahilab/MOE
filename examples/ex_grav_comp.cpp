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

enum state{
    to_neutral,
    stationary
};

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char* argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make options
    Options options("grav_comp.exe","Gravity Comp Demo");
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

    Time Ts = milliseconds(1);  // sample period for DAQ

    std::shared_ptr<MahiOpenExo> moe = nullptr;
    std::shared_ptr<Q8Usb> daq = nullptr;

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

    std::vector<std::vector<double>> setpoint_rad_ranges = {{-90 * DEG2RAD, 20 * DEG2RAD},
                                                            {-90 * DEG2RAD, 90 * DEG2RAD},
                                                            {-80 * DEG2RAD, 80 * DEG2RAD},
                                                            {-60 * DEG2RAD, 60 * DEG2RAD}};

                                     // state 0    // state 1    // state 2    // state 3    // state 4    // state 5    // state 6
    
    std::vector<Time> state_times = {seconds(2.0),seconds(30.0)};
    double t = 0;

    Time mj_Ts = milliseconds(50);

    std::vector<double> ref;

    WayPoint neutral_point = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});

    Timer timer(Ts, Timer::Hybrid);
    timer.set_acceptable_miss_rate(0.05);

    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(0.5);

    std::vector<std::string> dof_str = {"ElbowFE", "WristPS", "WristFE", "WristRU"};

    state current_state = to_neutral;
    WayPoint current_position;
    WayPoint new_position;
    Time traj_length;
    WayPoint dummy_waypoint = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});
    MinimumJerk mj(mj_Ts, dummy_waypoint, neutral_point.set_time(state_times[to_neutral]));
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

    mj.set_endpoints(start_pos, neutral_point.set_time(state_times[to_neutral]));
    UserParams badParams = {3,   // forearm position from the distal end of the robot
                                   7,   // cw position
                                   2}; // angle in degrees
    moe->set_user_parameters(badParams);
    // moe->moe_dynamic_model.add_arm_props("C:/Git/fes-exo-adl-traj-experiment/deidentified_data/S" + std::to_string(3092) + "/Arm_Cal",true);

    while (!stop) {
        moe->daq_read_all();

        moe->update();

        if (current_state == to_neutral) {
            ref = mj.trajectory().at_time(ref_traj_clock.get_elapsed_time());
        }
        else {
            ref = {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD};
        }
        

        for (std::size_t i = 0; i < moe->n_j; ++i) {
            ref[i] = clamp(ref[i], setpoint_rad_ranges[i][0], setpoint_rad_ranges[i][1]);
        }
        
        // calculate anatomical command torques
        if (result.count("no_torque") > 0){
            command_torques = {0.0, 0.0, 0.0, 0.0, 0.0};
            moe->set_raw_joint_torques(command_torques);
        }
        else{
            if (current_state == to_neutral){
            command_torques = moe->set_pos_ctrl_torques(ref);
            }
            else {
                command_torques = moe->calc_grav_torques();
                moe->set_raw_joint_torques(command_torques);
                
            }
        }

        if (ref_traj_clock.get_elapsed_time() > state_times[current_state]){
            switch (current_state) {
                case to_neutral:
                    current_state = stationary;
                    ref_traj_clock.restart();
                    break;
                case stationary:
                    stop = true;
                    break;
            }
        }

        std::vector<double> act_torque;
        if (!result.count("virtual")) {
            daq->AI.read();
            act_torque = {daq->AI[0],daq->AI[1],daq->AI[2],daq->AI[3]};

        }
        else{
            act_torque = {0,0,0,0};
        }
        std::vector<double> grav_torques =  moe->calc_grav_torques();
        data_line.clear();
        data_line.push_back(t);
        for (const auto &i : ref) data_line.push_back(i);
        for (const auto &i : moe->get_joint_positions()) data_line.push_back(i);
        for (const auto &i : moe->get_joint_velocities()) data_line.push_back(i);
        for (const auto &i : moe->get_joint_command_torques()) data_line.push_back(i);
        for (const auto &trq : act_torque) data_line.push_back(trq);
        for (const auto &trq : grav_torques) data_line.push_back(trq);
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
                                       "EFE trq (Nm)", "FPS trq (Nm)", "WFE trq (Nm)", "WRU trq (Nm)",
                                       "EFE act trq (Nm)","FPS act trq (Nm)", "WFE act trq (Nm)", "WRU act trq (Nm)",
                                       "EFE Grav","FPS Grav","WFE Grav","WRU Grav"};

    csv_write_row("data/rom_demo_results.csv",header);
    csv_append_rows("data/rom_demo_results.csv",data);
    
    moe->daq_disable();
    moe->disable();

    disable_realtime();

    // clear console buffer
    while (get_key_nb() != 0);

    return 0;
}