#include <Moe/MahiOpenExo/MahiOpenExo.hpp>
#include <Moe/MahiOpenExo/Joint.hpp>
#include <Mahi/Daq/Quanser/Q8Usb.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Timing/Timer.hpp>
#include <iomanip>
#include <Mahi/Util/Print.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <iterator>
#include <random>
#include <chrono>
#include<ctime>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;
using namespace std;

namespace moe {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    MahiOpenExo::MahiOpenExo(MoeParameters parameters) :
        Device("mahi_open_exo"),
        params_(parameters)
    {
        for (int i = 0; i < n_j; i++) {
            m_joint_positions.push_back(0.0);
            m_joint_velocities.push_back(0.0);
            m_joint_torques.push_back(0.0);
        }
        
    }

    MahiOpenExo::~MahiOpenExo() {
        if (is_enabled()) {
            disable();
        }
    }

    bool MahiOpenExo::on_enable() {
        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            if (!(*it)->enable()){
                LOG(Error) << "Failed to enable joints. Disabling MOE.";
                disable();
                return false;
            }
        }
        return true;
    }

    bool MahiOpenExo::on_disable() {
        bool successful = true;
        for(auto it = moe_joints.begin(); it != moe_joints.end(); ++it){
            if (!(*it)->disable()){
                LOG(Error) << "Failed to disable joints. Take proper precautions before handling.";
                successful = false;
            }
        }
        return successful;
    }

    void MahiOpenExo::calibrate(volatile std::atomic<bool>& stop) {
        // to use this function, move all joints to their maximum value and run
        daq_enable();
        // std::vector<int32> encoder_offsets = { 0, 0, 0, 0};
        for (int i = 0; i < n_j; i++) {
            // position (rad) / (2*pi [rad/rev]) * resolution [counts/rev] / eta 
            daq_encoder_write(i, params_.pos_limits_max_[i] / (2*PI) * params_.encoder_res_[i] / params_.eta_[i]*4);
            // std::cout << "Joint " << i << ": " << params_.pos_limits_max_[i] / (2*PI) * params_.encoder_res_[i] / params_.eta_[i] << "\n";
        }
        daq_disable();
        stop = true;
    }

    double MahiOpenExo::noise_degradation(){ //qué es lo que tengo de devolver PREGUNTARLE A ULI //MANUELAAAAAAAA
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);

        std::normal_distribution<double> distribution (0.0,0.01);
        return distribution(generator);
    }

    double MahiOpenExo::noise_lowFreq(){
        double max_amplitude = 0.05;
        vector <float> frequencies{1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8};
        double noise = 0;
        for (int i = 0; i < frequencies.size(); i++) {
            noise = noise + sin(frequencies[i]*2*PI*clock()/CLOCKS_PER_SEC);
        }
        return max_amplitude*noise/frequencies.size();
    }

    void MahiOpenExo::update(){
        for (size_t i = 0; i < n_j; i++){
            // filters velocity if the joint uses software filter (only an option for hardware)
            moe_joints[i]->filter_velocity();

            // update the position and velocity variables a single time
            m_joint_positions[i] = moe_joints[i]->get_position()+noise_lowFreq();
            m_joint_velocities[i] = moe_joints[i]->get_velocity();
        }
        // update the state of moe_dynamic_model
        moe_dynamic_model.update(m_joint_positions,m_joint_velocities);
    }

    ///////////////////////// Mass Properties and Model Calculations /////////////////////////

    void MahiOpenExo::set_user_parameters(UserParams newParams){
        // This will also update the mass properties based on the user params
        moe_dynamic_model.set_user_params(newParams);
    }

    void MahiOpenExo::set_user_parameters_from_json(std::string json_path) {
        // This will also update the mass properties based on the user params
        moe_dynamic_model.set_user_params_from_json(json_path);
    }

    std::vector<double> MahiOpenExo::calc_grav_torques(){
        auto eig_G = moe_dynamic_model.get_G();
        return copy_eigvec_to_stdvec(eig_G);
    }

    ///////////////////////// SMOOTH REFERENCE TRAJECTORY CLASS AND INSTANCES /////////////////////////

    MahiOpenExo::SmoothReferenceTrajectory::SmoothReferenceTrajectory(std::vector<double> speed, std::vector<double> ref_pos, std::vector<bool> active_dofs) :
        speed_(speed),
        n_dof(speed.size()),
        ref_(ref_pos),
        m_ref_init(true),
        m_active_dofs(active_dofs)
        {
            n_dof = std::count(m_active_dofs.begin(), m_active_dofs.end(), true);
            m_is_valid = (n_dof == speed_.size() && n_dof == ref_.size());
        }

    void MahiOpenExo::SmoothReferenceTrajectory::start(std::vector<double> current_pos, Time current_time) {
        if (m_ref_init) {
            m_started = true;
            prev_ref_ = current_pos;
            start_time_ = current_time;
        }
        else {
            print("ERROR: Reference position was not initialized. Must provide reference position to start().");
        }
    }

    void MahiOpenExo::SmoothReferenceTrajectory::start(std::vector<double> ref_pos, std::vector<double> current_pos, Time current_time) {
        m_started = true;
        prev_ref_ = current_pos;
        ref_ = ref_pos;
        start_time_ = current_time;
    }

    void MahiOpenExo::SmoothReferenceTrajectory::set_ref(std::vector<double> ref_pos, Time current_time) {
        if (!m_started) {
            print("ERROR: Cannot Call set_ref() before start().");
        }
        else if (ref_pos.size() != n_dof){
            print("ERROR: ref_pos is wrong size for the trajectory.");
        }
        else {
            for (auto i = 0; i < ref_pos.size(); ++i) {
                prev_ref_[i] = calculate_smooth_ref(i, current_time);
            }
            ref_ = ref_pos;
            start_time_ = current_time;
        }
    }

    double MahiOpenExo::SmoothReferenceTrajectory::calculate_smooth_ref(std::size_t dof, Time current_time) {
        if (m_started) {
            if (ref_[dof] == prev_ref_[dof]) {
                return ref_[dof];
            }
            return prev_ref_[dof] + (ref_[dof] - prev_ref_[dof]) * (current_time.as_seconds() - start_time_.as_seconds()) * speed_[dof] / std::abs(ref_[dof] - prev_ref_[dof]);
        }
        else {
            print("ERROR: Must give reference point first.");
            return NAN;
        }
    }

    bool MahiOpenExo::SmoothReferenceTrajectory::is_reached(std::vector<double> current_position, std::vector<double> tolerance){
        std::vector<char> check_dofs(n_dof,1);
        return check_goal_pos(ref_, current_position, check_dofs, tolerance, false);
    }

    void MahiOpenExo::SmoothReferenceTrajectory::stop() {
        m_started = false;
    }    
    
    ///////////////////////// TORQUE SETTING FUNCTIONS /////////////////////////

    std::vector<double> MahiOpenExo::set_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& ref, Time current_time) {
        std::vector<double> command_torques(n_j, 0.0);

        size_t num_active = 0;

        for (size_t i = 0; i < n_j; i++){
            // if the dof is active, calculate the torque to use, else it remains 0
            if (ref.m_active_dofs[i]){
                // calculate the new reference position
                double smooth_ref = ref.calculate_smooth_ref(num_active, current_time);
                // calculate the new torque based on the reference position
                command_torques[i] = joint_pd_controllers_[i].calculate(smooth_ref, get_joint_position(i), 0, get_joint_velocity(i));

                num_active++;
            }
        }
        set_raw_joint_torques(command_torques);

        return command_torques;
    }
    std::vector<double> MahiOpenExo::calc_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active) {
        std::vector<double> command_torques(n_j,0.0);

        if(std::count(active.begin(),active.end(),true) != ref.size()){
            LOG(Error) << "Size of 'ref' param must equal number of true values in 'active' param (default 4). Calculating 0 torques.";
        }
        else if(size(active) != 4){
            LOG(Error) << "Size of 'active' param must be 4. Calculating 0 torques.";
        }
        else{
            size_t active_counter = 0;
            for (std::size_t i = 0; i < n_j; ++i) {
                if (active[i]){
                    command_torques[i] = joint_pd_controllers_[i].calculate(ref[active_counter],m_joint_positions[i],0,m_joint_velocities[i]);
                    active_counter++;
                }
            }
        }

        return command_torques;
    }
    std::vector<double> MahiOpenExo::set_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active) {
        std::vector<double> command_torques = calc_pos_ctrl_torques(ref,active);

        set_raw_joint_torques(command_torques);

        return command_torques;
    }

    void MahiOpenExo::set_raw_joint_torques(std::vector<double> new_torques) {
        
        // update reference
        m_joint_torques = new_torques;

        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            (*it)->set_torque(new_torques[it - moe_joints.begin()]);
        }
    }

    void MahiOpenExo::set_high_gains(std::vector<bool> active){
        std::vector<double> high_gains = {2.0,3.0,3.0,3.0};
        for (std::size_t i=0; i<n_j; ++i) {
            if (active[i]){
                joint_pd_controllers_[i].kp = gains_P[i]*high_gains[i];
                joint_pd_controllers_[i].kd = gains_D[i]*1.0;
            }
        }
    }

    void MahiOpenExo::set_normal_gains(std::vector<bool> active){
        for (std::size_t i=0; i<n_j; ++i) {
            if (active[i]){
                joint_pd_controllers_[i].kp = gains_P[i];
                joint_pd_controllers_[i].kd = gains_D[i];
            }
        }
    }

    /////////////////// GOAL CHECKING FUNCTIONS ///////////////////

    bool MahiOpenExo::check_joint_goal_pos(std::vector<double> goal_pos, std::vector<char> check_dof, bool print_output) const {
        return check_goal_pos(goal_pos, get_joint_positions(), check_dof, m_goal_err_tol, print_output);
    }

    /////////////////// LIMIT CHECKING ON THE MEII ///////////////////

    bool MahiOpenExo::any_limit_exceeded(){
        return (any_velocity_limit_exceeded() || any_torque_limit_exceeded());
    }

    bool MahiOpenExo::any_velocity_limit_exceeded(){
        bool exceeded = false;
        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            if ((*it)->velocity_limit_exceeded())
                exceeded = true;
        }
        return exceeded;
    }

    bool MahiOpenExo::any_torque_limit_exceeded(){
        bool exceeded = false;
        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            if ((*it)->torque_limit_exceeded())
                exceeded = true;
        }
        return exceeded;
    }

    //////////////// MISC USEFUL UTILITY FUNCTIONS ////////////////

    bool MahiOpenExo::check_goal_pos(std::vector<double> goal_pos, std::vector<double> current_pos, std::vector<char> check_dof, std::vector<double> error_tol, bool print_output) {

        assert((goal_pos.size() == current_pos.size()) && ((goal_pos.size() == check_dof.size())) && (goal_pos.size() == error_tol.size()));

        bool goal_reached = true;
        for (size_t i = 0; i < goal_pos.size(); ++i) {
            if (check_dof.at(i)) {
                if (std::abs(goal_pos[i] - current_pos[i]) > std::abs(error_tol[i])) {
                    if (print_output && goal_reached) {
                        std::cout << "Joint " << std::to_string(i) << " error is " << (std::abs(goal_pos[i] - current_pos[i])*RAD2DEG) << std::endl;
                    }
                    goal_reached = false;
                }
            }
        }
        return goal_reached;
    }

    std::vector<double> MahiOpenExo::copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec) {
        std::vector<double> std_vec(eigen_vec.size());
        for (int i = 0; i < eigen_vec.size(); ++i) {
            std_vec[i] = eigen_vec[i];
        }
        return std_vec;
    }

    Eigen::VectorXd MahiOpenExo::copy_stdvec_to_eigvec(const std::vector<double>& std_vec) {
        Eigen::VectorXd eigen_vec(std_vec.size());
        for (size_t i = 0; i < std_vec.size(); ++i) {
            eigen_vec[i] = std_vec[i];
        }
        return eigen_vec;
    }

    void MahiOpenExo::calibrate_auto(volatile std::atomic<bool>& stop){
        mahi::util::print("Begining Calibration");
        // destinations for the joints after setting calibration
        std::array<double,n_j> neutral_points = {-35*DEG2RAD, 0, 0, 0};
        
        // create needed variables
        std::vector<double> zeros = { 0, 0, 0, 0}; // determined zero positions for each joint
        std::array<int, n_j> dir = { -1 , 1, -1, -1 };  // direction to rotate each joint
        uint32 calibrating_joint = 0;               // joint currently calibrating
        bool returning = false;                     // bool to track if calibrating joint is return to zero
        double pos_ref = 0;                         // desired position
        
        std::array<double,n_j> multiplier = {4.0, 4.0, 4.0, 4.0};

        // std::array<double, 5> vel_ref = {30 * DEG2RAD, 30 * DEG2RAD, 0.01, 0.01, 0.01}; // desired velocities
        double vel_ref = 20 * DEG2RAD;

        std::vector<double> stored_positions;  // stores past positions
        stored_positions.reserve(100000);

        // std::array<double, n_j> sat_torques = { 4.0, 4.0, 2.0, 2.0}; // temporary saturation torques
        std::array<double, n_j> sat_torques = { 6.0, 2.0, 2.0, 1.0}; // temporary saturation torques

        Time timeout = seconds(60); // max amout of time we will allow calibration to occur for

        // enable DAQs, zero encoders, and start watchdog
        daq_enable();
        for (size_t i = 0; i < n_j; i++){
            daq_encoder_write((int32)i,0);
        }
        // daq_watchdog_start();

        // enable MEII
        enable();

        daq_read_all();
        update();

        zeros = get_joint_positions();
        pos_ref = zeros[calibrating_joint];

        // start the clock
        Timer timer(milliseconds(1), Timer::WaitMode::Hybrid);
        
        // start the calibration control loop
        while (!stop && timer.get_elapsed_time() < timeout) {

            // read and reload DAQs
            daq_read_all();
            update();
            // daq_watchdog_kick();

                // iterate over all joints
            for (std::size_t i = 0; i < n_j; i++) {

                // get positions and velocities
                double pos_act = get_joint_position(i);
                double vel_act = get_joint_velocity(i);

                double torque = 0;
                if (i == calibrating_joint) {
                    if (!returning) {

                        // calculate torque req'd to move the calibrating joint forward at constant speed
                        pos_ref += dir[i] * vel_ref * timer.get_period().as_seconds();
                        torque = joint_pd_controllers_[i].calculate(pos_ref, pos_act, 0, vel_act);
                        torque = clamp(torque, sat_torques[i]);
                        // mahi::util::print("pos_ref: {}, torque: {}",pos_ref,torque);

                        // check if the calibrating joint is still moving
                        stored_positions.push_back(pos_act);
                        bool moving = true;
                        if (stored_positions.size() > 100) {
                            moving = false;
                            for (size_t j = stored_positions.size() - 100; j < stored_positions.size(); j++) {
                                moving = stored_positions[j] != stored_positions[j - 1];
                                if (moving)
                                    break;
                            }
                        }

                        // if it's not moving, it's at a hardstop so record the position and deduce the zero location
                        if (!moving) {
                            auto extreme = (dir[i] == 1) ? params_.pos_limits_max_[i] : params_.pos_limits_min_[i];
                            daq_encoder_write((int)i,extreme / (2*PI) * params_.encoder_res_[i] / params_.eta_[i] *multiplier[i]);
                            returning = true;
                            // update the reference position to be the current one
                            pos_ref = extreme;
                        }
                    }

                    else {
                        // calculate torque req'd to retur the calibrating joint back to zero
                        pos_ref -= dir[i] * vel_ref *  timer.get_period().as_seconds();
                        torque = joint_pd_controllers_[i].calculate(pos_ref, pos_act, 0, vel_act);
                        torque = clamp(torque, sat_torques[i]);


                        if (dir[i] * pos_ref <= dir[i] * neutral_points[i]) {
                            std::cout << pos_ref << std::endl;
                            // reset for the next joint
                            calibrating_joint += 1;
                            pos_ref = zeros[calibrating_joint];
                            returning = false;
                            LOG(Info) << "Joint " << moe_joints[i]->get_name() << " calibrated";
                            if (calibrating_joint == n_j) stop = true;
                        }
                    }
                }
                else {
                    // lock all other joints at their zero positions
                    if (i > calibrating_joint){
                        torque = joint_pd_controllers_[i].calculate(zeros[i], pos_act, 0, vel_act);
                    }
                    else{
                        torque = joint_pd_controllers_[i].calculate(neutral_points[i], pos_act, 0, vel_act);
                    }
                    torque = clamp(torque, sat_torques[i]);
                }
                // print("joint {} - pos: {}, ref: {}, torque: {}", i, pos_act, pos_ref, torque);
                moe_joints[i]->set_torque(torque);
            }
            
            // write all DAQs
            daq_write_all();

            for (const auto &vel_safety_check : get_joint_velocities()) {
                if (abs(vel_safety_check) > vel_ref*4.0){
                    stop = true;
                }
            }
            
            if (any_velocity_limit_exceeded() || any_torque_limit_exceeded()) {
                stop = true;
            }

            // wait the clock
            timer.wait();
        }

        // disable MEII
        disable();

        // disable DAQ
        daq_disable();
    }
}