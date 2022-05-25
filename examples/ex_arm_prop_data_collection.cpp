// This code is an example of data collection to find the arm properties. You can copy this into your repository if using for a project


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
    gather_data
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
    Options options("ex_arm_prop_data_collection.exe","Arm property data collection example");
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

    Time Ts = milliseconds(1); // sample period for DAQ

    std::shared_ptr<MahiOpenExo> moe = nullptr;
    std::shared_ptr<Q8Usb> daq = nullptr;

    if(result.count("virtual") > 0){
        MoeConfigurationVirtual config_vr;
        moe = std::make_shared<MahiOpenExoVirtual>(config_vr);
    }
    else {
        daq = std::make_shared<Q8Usb>();
        daq->open();

        MoeConfigurationHardware config_hw(*daq,VelocityEstimator::Hardware);
        std::vector<TTL> idle_values(8,TTL_LOW);
        daq->DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
        daq->DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
        daq->DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);

        moe = std::make_shared<MahiOpenExoHardware>(config_hw);
    }

    // calibrate
    if (result.count("calibrate") > 0) {
        moe->calibrate_auto(stop);
        LOG(Info) << "Mahi Open Exo encoders calibrated";
        return 0;
    }

    // make MelShares
    MelShare ms_pos("ms_pos");
    MelShare ms_vel("ms_vel");
    MelShare ms_trq("ms_trq");
    MelShare ms_ref("ms_ref");

    std::vector<Time> state_times = {seconds(2.0),seconds(10.0)};
    // set up for chirp 
    ////////////////////////////////EFE,FPS,WFE,WRU
    std::vector<double> min_freq   = {0.1,0.1,0.1,0.1};
    std::vector<double> max_freq   = {.75,1.5,1.5,1.5};
    std::vector<double> amplitudes = {20.0,20.0,20.0,10.0};
    std::vector<double> c_const(4,0.0);
    for (std::size_t i = 0; i < 4; i++) {
        c_const[i] = (max_freq[i]-min_freq[i])/state_times[gather_data].as_seconds();
    }


    std::vector<std::vector<double>> setpoint_rad_ranges = {{-90 * DEG2RAD, 20 * DEG2RAD},
                                                            {-90 * DEG2RAD, 90 * DEG2RAD},
                                                            {-80 * DEG2RAD, 80 * DEG2RAD},
                                                            {-60 * DEG2RAD, 60 * DEG2RAD}};


    double t = 0;

    Time mj_Ts = milliseconds(50);

    std::vector<double> ref;

    WayPoint neutral_point = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD});

    Timer timer(Ts, Timer::Hybrid);
    timer.set_acceptable_miss_rate(0.05);

    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(0.5);

    std::vector<std::string> dof_str = {"WRU", "WFE", "FPS", "EFE"};

    state current_state = to_neutral;

    WayPoint current_position;
    WayPoint new_position;
    Time traj_length;
    WayPoint dummy_waypoint = WayPoint(Time::Zero, {0,0,0,0});
    MinimumJerk mj(mj_Ts, dummy_waypoint, neutral_point.set_time(state_times[to_neutral]));
    std::vector<double> traj_max_diff = {60 * DEG2RAD, 60 * DEG2RAD, 100 * DEG2RAD, 60 * DEG2RAD};
    mj.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);
    Clock ref_traj_clock;

    std::vector<double> aj_positions(4,0.0);
    std::vector<double> aj_velocities(4,0.0);

    std::vector<double> command_torques(4,0.0);

    ref_traj_clock.restart();

    // enable DAQ and exo
    moe->daq_enable();

    moe->enable();

    LOG(Info) << "Starting Movement.";

    std::vector<std::vector<std::vector<double>>> data3D;
    std::vector<std::vector<double>> data2D;
    std::vector<double> data_line;

    moe->daq_read_all();
    moe->update();

    WayPoint start_pos(Time::Zero, moe->get_joint_positions());

    mj.set_endpoints(start_pos, neutral_point.set_time(state_times[to_neutral]));

    // moe->moe_dynamic_model.set_user_params(UserParams{3,4,0});
    moe->set_user_parameters(UserParams{3,4,0});
    int active_joint = 3;
    while (!stop) {
        moe->daq_read_all();

        moe->update();

        if (current_state == to_neutral) {
            ref = mj.trajectory().at_time(ref_traj_clock.get_elapsed_time());
        }
        else {
            for(std::size_t i = 0; i < moe->n_j; ++i) {
                if (i == active_joint) {
                    ref[i] = neutral_point[i] + amplitudes[i]*DEG2RAD*sin(2*PI*(c_const[active_joint]/2.0*ref_traj_clock.get_elapsed_time().as_seconds()*ref_traj_clock.get_elapsed_time().as_seconds() + min_freq[active_joint]*ref_traj_clock.get_elapsed_time().as_seconds()));
                }
                else {
                    ref[i] = neutral_point[i];
                }
            }
        }

        for (std::size_t i = 0; i < moe->n_j; ++i) {
            ref[i] = clamp(ref[i], setpoint_rad_ranges[i][0], setpoint_rad_ranges[i][1]);
        }

        if (result.count("no_torque") > 0){
            command_torques = {0.0, 0.0, 0.0, 0.0};
            moe->set_raw_joint_torques(command_torques);
        }
        else{
            command_torques = moe->set_pos_ctrl_torques(ref);
        }

        if (current_state == gather_data) {
            data_line.clear();
            data_line.push_back(ref_traj_clock.get_elapsed_time().as_seconds());
            for (const auto &i : ref) data_line.push_back(i);
            for (const auto &i : moe->get_joint_positions()) data_line.push_back(i);
            for (const auto &i : moe->get_joint_velocities()) data_line.push_back(i);
            for (const auto &i : command_torques) data_line.push_back(i);
            data2D.push_back(data_line);
        }

        if (ref_traj_clock.get_elapsed_time() > state_times[current_state]) {
            switch (current_state) {
                case to_neutral:
                    current_state = gather_data;
                    ref_traj_clock.restart();
                    break;
                case gather_data:
                    current_state = to_neutral;
                    start_pos.set_pos(moe->get_joint_positions());
                    mj.set_endpoints(start_pos,neutral_point.set_time(state_times[to_neutral]));
                    ref_traj_clock.restart();
                    data3D.push_back(data2D);
                    data2D.clear();
                    if (active_joint != 0) {
                        active_joint--;
                    }
                    else {
                        stop = true;
                    }
                    break;
            }
        }

        

        if (moe->any_limit_exceeded()) {
            stop = true;
        }

        if (!stop) moe->daq_write_all();

        ms_ref.write_data(moe->get_joint_positions());
        ms_pos.write_data(moe->get_joint_velocities());

        t = timer.wait().as_seconds();
    }

    command_torques = {0.0, 0.0, 0.0, 0.0};
    moe->set_raw_joint_torques(command_torques);
    moe->daq_write_all();

    moe->daq_disable();
    moe->disable();

    std::vector<std::string> header = {"Time (s)",
                                    "EFERef_rad_","FPSRef_rad_","WFERef_rad_","WRURef_rad_",
                                    "EFEAct_rad_","FPSAct_rad_","WFEAct_rad_","WRUAct_rad_",
                                    "EFEAct_rad_s_","FPSAct_rad_s_","WFEAct_rad_s_","WRUAct_rad_s_",
                                    "EFETrq_Nm_","FPSTrq_Nm_","WFETrq_Nm_","WRUTrq_Nm_"};

    for (std::size_t i = 0; i < moe->n_j; ++i) {
        std::string filename = "data/" + dof_str[i] + "/data.csv";
        csv_write_row(filename,header);
        csv_append_rows(filename,data3D[i]);
    }
    // csv_write_row("data/WRU/data.csv",header);
    // csv_append_rows("data/WRU/data.csv",data3D[0]);


}
