#include<Mahi/Daq.hpp>
#include<Mahi/Util.hpp>
#include<Mahi/Robo.hpp>
#include<common.hpp>

using namespace mahi::daq;
using namespace mahi::util;
using namespace mahi::robo;

using std::vector;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[])
{
    register_ctrl_handler(handler);

    Options options("moe_test.exe","Test of the Combined OW plus Elbow");
    options.add_options()
        ("c,calibrate","Calibration of all of the MOE Joints");
    
    auto result = options.parse(argc, argv);

    Q8Usb q8;
    if (!q8.is_open())
        return 1;

    // if we input calibrate, zero all encoders and exit
    if (result.count("calibrate") > 0){
        prompt("Press ENTER when the:\n\tElbow is fully flexed\n\tForearm is fully pronated\n\tWrist FE is fully Flexed\n\tWrist RU is fully Radially deviated");
        for (auto i = 0; i < moe_n_dof; i++){
            q8.encoder.zero(i);
        }
        return 0;
    }

    // EXPERIMENT PARAMETERS
    vector<bool> active_joints = {true, true, true, true};

    // trajectory parameters -> desired_pos = traj_amplitude*sin(2.0*PI*traj_frequency*t.as_seconds())
    vector<double> traj_frequencies = {          0.5,           0.5,           0.5,           0.5}; // Hz
    vector<double> traj_amplitudes =  { 20.0*DEG2RAD,  20.0*DEG2RAD,  20.0*DEG2RAD,  15.0*DEG2RAD}; // amplitude in radians
    vector<double> traj_offsets =     {-35.0*DEG2RAD, -90.0*DEG2RAD, -70.0*DEG2RAD, -35.0*DEG2RAD}; // sinwave offset
    vector<double> pd_scales = {0.25, 0.05, 0.05, 0.05};
    Time traj_time = 8_s;
    Time go_to_start_time = 5_s;
    // END EXPERIMENT PARAMETERS

    // container to store data
    vector<vector<double>> data;
    

    // setup adjusted vectors that only take into account active joints
    vector<unsigned int> active_map;
    vector<double> active_torque_limits;
    vector<double> active_velocity_limits;

    for (auto i = 0; i < moe_n_dof; i++){
        if (active_joints[i]) {
            active_map.push_back(i);

            active_torque_limits.push_back(torque_limits[i]);
            active_velocity_limits.push_back(velocity_limits[i]);
        }
    }
    size_t num_active_dof = active_map.size();

    // initialize q8
    for (auto i = 0; i < moe_n_dof; i++){
        // initialize DO
        q8.DO.enable_values[i]  = TTL_LOW;
        q8.DO.disable_values[i] = TTL_LOW;

        // initialize AO
        q8.AO.enable_values[i]  = 0;
        q8.AO.disable_values[i] = 0;
        
        // initialize encoders
        q8.encoder.units[i] = (2.0*PI/encoder_cprs[i])*gear_ratios[i];
    }

    // pd controllers used for each joint
    vector<PdController> joint_controllers;
    for (size_t i = 0; i < moe_n_dof; i++){
        joint_controllers.emplace_back(Kps[i]*pd_scales[i],Kds[i]*pd_scales[i]);
    }

    q8.enable();
    sleep(1_ms);
    q8.read_all();

    vector<double> desired_pos (num_active_dof,0);
    vector<double> desired_vel (num_active_dof,0);
    vector<double> current_pos (num_active_dof,0);
    vector<double> current_vel (num_active_dof,0);
    vector<double> desired_trq (num_active_dof,0);
    vector<double> initial_pos (num_active_dof,0);
    
    // get initial position to use for getting to our starting position for trajectories
    for (size_t i = 0; i < num_active_dof; i++){
        initial_pos[i] = q8.encoder.positions[active_map[i]];
    }

    // wait for the user to hit enter before starting
    prompt("Press ENTER to begin.");
    
    // enable all joints
    for (auto i = 0; i < num_active_dof; i++){
        q8.DO[active_map[i]] = TTL_HIGH;
    }
    
    // double desiredTorque = 0.0; // Nm
    Time t = Time::Zero;
    Timer timer(1000_Hz);
    while (!stop && t < traj_time+go_to_start_time) {
        q8.read_all();

        for (auto i = 0; i < num_active_dof; i++){
            auto moe_joint = active_map[i];

            // get desired position either of being to the start pos, or as part of trajectory
            if (t < go_to_start_time){  
                desired_pos[i] = interp(t.as_seconds(),0.0,go_to_start_time.as_seconds(),initial_pos[i],traj_offsets[moe_joint]+traj_amplitudes[moe_joint]);
            }
            else{
                desired_pos[i] = traj_amplitudes[moe_joint]*cos(2.0*PI*traj_frequencies[moe_joint]*(t-go_to_start_time).as_seconds()) + traj_offsets[moe_joint];
            }

            // update pos and velocity from the daq
            current_pos[i] = q8.encoder.positions[moe_joint];
            current_vel[i] = q8.velocity.velocities[moe_joint];

            desired_vel[i] = 0.0;

            // calc desired torque
            desired_trq[i] = joint_controllers[moe_joint].calculate(desired_pos[i],current_pos[i],desired_vel[i],current_vel[i]);  

            // compute the voltage out based on desired torque
            double commandedCurrent = desired_trq[i]/Kts[moe_joint]; // Nm / (Nm / A) = [Nm]
            double volts_out = (switch_dirs[moe_joint] ? -1.0 : 1.0)*commandedCurrent/amp_ratios[moe_joint]; // A / (A / V) = [V]
            q8.AO[moe_joint] = volts_out;      
        }

        
        // check if we have tripped any of our safety limits
        if (torque_limits_exceeded(desired_trq,active_torque_limits) || 
            velocity_limits_exceeded(current_vel,active_velocity_limits)){
            stop = true;
        }

        if (!q8.watchdog.kick()){
            LOG(Error) << "Failed to kick watchdog. Exiting code." << std::endl;
            stop = true;
        }

        q8.write_all();
        
        // store data
        std::vector<double> data_line;
        data_line.push_back(t.as_seconds());
        for(const auto& pos : current_pos)
            data_line.push_back(pos);
        for(const auto& vel : current_vel)
            data_line.push_back(vel);
        for(const auto& pos : desired_pos)
            data_line.push_back(pos);
        for(const auto& vel : desired_vel)
            data_line.push_back(vel);
        for(const auto& trq : desired_trq)
            data_line.push_back(trq);
        data.push_back(data_line);
        
        t = timer.wait();
    }
    // disable the joints
    for (auto i = 0; i < num_active_dof; i++){
        q8.DO[active_map[i]] = TTL_LOW;
    }

    q8.disable();
    q8.close();

    std::vector<std::string> header = {"Time (s)",
                                       "EFE pos (rad)", "FPS pos (rad)", "WFE pos (rad)", "WRU pos (rad)",
                                       "EFE vel (rad/s)", "FPS vel (rad/s)", "WFE vel (rad/s)", "WRU vel (rad/s)",
                                       "EFE des pos (rad)", "FPS des pos (rad)", "WFE des pos (rad)", "WRU des pos (rad)",
                                       "EFE des vel (rad/s)", "FPS des vel (rad/s)", "WFE des vel (rad/s)", "WRU des vel (rad/s)",
                                       "EFE trq (Nm)", "FPS trq (Nm)", "WFE trq (Nm)", "WRU trq (Nm)"};

    
    // remove header entries that aren't active. this is a dumb way, but couldn't think of a better way at the moment
    int cnt = 0;
    auto it = header.begin();
    while(it != header.end()){
        if (cnt != 0 && !active_joints[(cnt-1) % moe_n_dof])
            it = header.erase(it);
        else
            it++;  
        
        cnt++;
    }
    

    Timestamp ts;
    std::string filepath = "moe_testing/data_" + ts.yyyy_mm_dd_hh_mm_ss() + ".csv";
    csv_write_row(filepath,header);
    csv_append_rows(filepath,data);
    return 0;
}