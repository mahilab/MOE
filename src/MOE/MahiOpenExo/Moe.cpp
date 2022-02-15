#include <Moe/MahiOpenExo/Moe.hpp>
#include <Moe/MahiOpenExo/Joint.hpp>
#include <Mahi/Daq/Quanser/Q8Usb.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Timing/Timer.hpp>
#include <iomanip>
#include <Mahi/Util/Print.hpp>
#include <Mahi/Util/Logging/Log.hpp>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;

namespace moe {

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    Moe::Moe(MoeParameters parameters) :
        Device("mahi_open_exo"),
        params_(parameters)
    {

        for (int i = 0; i < n_j; i++) {
            m_joint_positions.push_back(0.0);
            m_joint_velocities.push_back(0.0);
            m_joint_torques.push_back(0.0);
        }
        
    }

    Moe::~Moe() {
        if (is_enabled()) {
            disable();
        }
    }

    bool Moe::on_enable() {
        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            if (!(*it)->enable()){
                LOG(Error) << "Failed to enable joints. Disabling MOE.";
                disable();
                return false;
            }
        }
        return true;
    }

    bool Moe::on_disable() {
        bool successful = true;
        for(auto it = moe_joints.begin(); it != moe_joints.end(); ++it){
            if (!(*it)->disable()){
                LOG(Error) << "Failed to disable joints. Take proper precautions before handling.";
                successful = false;
            }
        }
        return successful;
    }

    void Moe::calibrate(volatile std::atomic<bool>& stop) {
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

    void Moe::update(){
        for (size_t i = 0; i < n_j; i++){
            // filters velocity if the joint uses software filter (only an option for hardware)
            moe_joints[i]->filter_velocity();

            // update the position and velocity variables a single time
            m_joint_positions[i] = moe_joints[i]->get_position();
            m_joint_velocities[i] = moe_joints[i]->get_velocity();
        }
    }

    ///////////////////////// Mass Properties and Model Calculations /////////////////////////

    double Moe::get_forearm_dist(){
        double forearm_distance = 0.28024875 - (sub_params.forearm_location-3)*0.005;
        return forearm_distance;
    }

    void Moe::set_subject_parameters(SubjectParameters newParams){
        sub_params = newParams;
        update_J0();
    }

    SubjectParameters Moe::get_subject_parameters(){
        return sub_params;
    }

    void Moe::update_J0(){
        double x_cw = cw_mass_props.Pcx;
        double y_cw = cw_mass_props.Pcy + (sub_params.cw_location - 4)*0.01270000;
        // double z_cw = cw_mass_props.Pcz;
        double x_sl = sl_mass_props.Pcx;
        double y_sl = sl_mass_props.Pcy + (sub_params.forearm_location-3)*0.005;
        // double z_sl = sl_mass_props.Pcz;
        double newPcy = (cw_mass_props.m*y_cw + sl_mass_props.m*y_sl + el_mass_props.m*el_mass_props.Pcy)/(cw_mass_props.m + sl_mass_props.m + el_mass_props.m);
        // Parallel Axis Thm
        double Iczz_cw = cw_mass_props.Iczz + cw_mass_props.m*((x_cw-moe_mass_props.all_props[0].Pcx)*(x_cw-moe_mass_props.all_props[0].Pcx)+(y_cw-newPcy)*(y_cw-newPcy));
        double Iczz_sl = sl_mass_props.Iczz + sl_mass_props.m*((x_sl-moe_mass_props.all_props[0].Pcx)*(x_sl-moe_mass_props.all_props[0].Pcx)+(y_sl-newPcy)*(y_sl-newPcy));
        double Iczz_el = el_mass_props.Iczz + el_mass_props.m*((el_mass_props.Pcx-moe_mass_props.all_props[0].Pcx)*(el_mass_props.Pcx-moe_mass_props.all_props[0].Pcx)+(el_mass_props.Pcy-newPcy)*(el_mass_props.Pcy-newPcy));
        // moe_mass_props.all_props[0].Pcx = ;
        moe_mass_props.J0.Pcy = newPcy;
        // moe_mass_props.all_props[0].Pcz = ;
        // moe_mass_props.all_props[0].Icxx = ;
        // moe_mass_props.all_props[0].Icyy = ;
        moe_mass_props.J0.Iczz = Iczz_cw + Iczz_sl + Iczz_el;
        // moe_mass_props.all_props[0].Icxy = ;
        // moe_mass_props.all_props[0].Icxz = ;
        // moe_mass_props.all_props[0].Icyz = ;
    }

    std::vector<double> Moe::calc_grav_torques(){
        double q0 = m_joint_positions[0];
        double q1 = m_joint_positions[1];
        double q2 = m_joint_positions[2];
        double q3 = m_joint_positions[3];
        double q_s = DEG2RAD*sub_params.shoulder_ang;
        double d = get_forearm_dist();
        
        double m0_ = moe_mass_props.J0.m; double m1_ = moe_mass_props.J1.m; double m2_ = moe_mass_props.J2.m; double m3_ = moe_mass_props.J3.m; 
        double Pcx0_ = moe_mass_props.J0.Pcx; double Pcx1_ = moe_mass_props.J1.Pcx; double Pcx2_ = moe_mass_props.J2.Pcx; double Pcx3_ = moe_mass_props.J3.Pcx;
        double Pcy0_ = moe_mass_props.J0.Pcy; double Pcy1_ = moe_mass_props.J1.Pcy; double Pcy2_ = moe_mass_props.J2.Pcy; double Pcy3_ = moe_mass_props.J3.Pcy; 
        double Pcz0_ = moe_mass_props.J0.Pcz; double Pcz1_ = moe_mass_props.J1.Pcz; double Pcz2_ = moe_mass_props.J2.Pcz; double Pcz3_ = moe_mass_props.J3.Pcz; 
        double Icxx0_ = moe_mass_props.J0.Icxx; double Icxx1_ = moe_mass_props.J1.Icxx; double Icxx2_ = moe_mass_props.J2.Icxx; double Icxx3_ = moe_mass_props.J3.Icxx; 
        double Icyy0_ = moe_mass_props.J0.Icyy; double Icyy1_ = moe_mass_props.J1.Icyy; double Icyy2_ = moe_mass_props.J2.Icyy; double Icyy3_ = moe_mass_props.J3.Icyy; 
        double Iczz0_ = moe_mass_props.J0.Iczz; double Iczz1_ = moe_mass_props.J1.Iczz; double Iczz2_ = moe_mass_props.J2.Iczz; double Iczz3_ = moe_mass_props.J3.Iczz; 
        double Icxy0_ = moe_mass_props.J0.Icxy; double Icxy1_ = moe_mass_props.J1.Icxy; double Icxy2_ = moe_mass_props.J2.Icxy; double Icxy3_ = moe_mass_props.J3.Icxy; 
        double Icxz0_ = moe_mass_props.J0.Icxz; double Icxz1_ = moe_mass_props.J1.Icxz; double Icxz2_ = moe_mass_props.J2.Icxz; double Icxz3_ = moe_mass_props.J3.Icxz; 
        double Icyz0_ = moe_mass_props.J0.Icyz; double Icyz1_ = moe_mass_props.J1.Icyz; double Icyz2_ = moe_mass_props.J2.Icyz; double Icyz3_ = moe_mass_props.J3.Icyz; 

        double t2 = cos(q0);
        double t3 = cos(q1);
        double t4 = cos(q2);
        double t5 = cos(q3);
        double t6 = cos(q_s);
        double t7 = sin(q0);
        double t8 = sin(q1);
        double t9 = sin(q2);
        double t10 = sin(q3);
        double t11 = sin(q_s);
        double tau0 = g*t6*(-Pcx0_*m0_*t7-Pcy0_*m0_*t2-Pcz1_*m1_*t2+d*m1_*t2+d*m2_*t2+d*m3_*t2-Pcx2_*m2_*t2*t4+Pcx1_*m1_*t7*t8+Pcy1_*m1_*t3*t7+Pcy2_*m2_*t2*t9-Pcz2_*m2_*t3*t7+Pcz3_*m3_*t2*t9-Pcx3_*m3_*t2*t4*t5+Pcx3_*m3_*t3*t7*t10-Pcx2_*m2_*t7*t8*t9+Pcy3_*m3_*t3*t5*t7+Pcy3_*m3_*t2*t4*t10-Pcy2_*m2_*t4*t7*t8-Pcz3_*m3_*t4*t7*t8-Pcx3_*m3_*t5*t7*t8*t9+Pcy3_*m3_*t7*t8*t9*t10);
        double tau1 = g*(Pcx1_*m1_*t8*t11+Pcy1_*m1_*t3*t11-Pcz2_*m2_*t3*t11-Pcx1_*m1_*t2*t3*t6+Pcx3_*m3_*t3*t10*t11-Pcx2_*m2_*t8*t9*t11+Pcy1_*m1_*t2*t6*t8+Pcy3_*m3_*t3*t5*t11-Pcy2_*m2_*t4*t8*t11-Pcz2_*m2_*t2*t6*t8-Pcz3_*m3_*t4*t8*t11+Pcx2_*m2_*t2*t3*t6*t9+Pcx3_*m3_*t2*t6*t8*t10-Pcx3_*m3_*t5*t8*t9*t11+Pcy2_*m2_*t2*t3*t4*t6+Pcy3_*m3_*t2*t5*t6*t8+Pcy3_*m3_*t8*t9*t10*t11+Pcz3_*m3_*t2*t3*t4*t6+Pcx3_*m3_*t2*t3*t5*t6*t9-Pcy3_*m3_*t2*t3*t6*t9*t10);
        double tau2 = g*(Pcx2_*m2_*t3*t4*t11+Pcx2_*m2_*t6*t7*t9+Pcy2_*m2_*t4*t6*t7-Pcy2_*m2_*t3*t9*t11+Pcz3_*m3_*t4*t6*t7-Pcz3_*m3_*t3*t9*t11+Pcx2_*m2_*t2*t4*t6*t8+Pcx3_*m3_*t3*t4*t5*t11+Pcx3_*m3_*t5*t6*t7*t9-Pcy2_*m2_*t2*t6*t8*t9-Pcy3_*m3_*t3*t4*t10*t11-Pcy3_*m3_*t6*t7*t9*t10-Pcz3_*m3_*t2*t6*t8*t9+Pcx3_*m3_*t2*t4*t5*t6*t8-Pcy3_*m3_*t2*t4*t6*t8*t10);
        double tau3 = -g*m3_*(-Pcx3_*t5*t8*t11+Pcy3_*t8*t10*t11+Pcx3_*t2*t3*t5*t6-Pcx3_*t4*t6*t7*t10+Pcx3_*t3*t9*t10*t11-Pcy3_*t2*t3*t6*t10-Pcy3_*t4*t5*t6*t7+Pcy3_*t3*t5*t9*t11+Pcx3_*t2*t6*t8*t9*t10+Pcy3_*t2*t5*t6*t8*t9);
        std::vector<double> grav_torques{tau0,tau1,tau2,tau3};
        return grav_torques;
    }
    // void Moe::initialize_parameters(SubjectParameters subject_params){
    //     sub_params = subject_params;
    //     // And or is this what would calculate the mass props?
    // }
    ///////////////////////// SMOOTH REFERENCE TRAJECTORY CLASS AND INSTANCES /////////////////////////

    Moe::SmoothReferenceTrajectory::SmoothReferenceTrajectory(std::vector<double> speed, std::vector<double> ref_pos, std::vector<bool> active_dofs) :
        speed_(speed),
        n_dof(speed.size()),
        ref_(ref_pos),
        m_ref_init(true),
        m_active_dofs(active_dofs)
        {
            n_dof = std::count(m_active_dofs.begin(), m_active_dofs.end(), true);
            m_is_valid = (n_dof == speed_.size() && n_dof == ref_.size());
        }

    void Moe::SmoothReferenceTrajectory::start(std::vector<double> current_pos, Time current_time) {
        if (m_ref_init) {
            m_started = true;
            prev_ref_ = current_pos;
            start_time_ = current_time;
        }
        else {
            print("ERROR: Reference position was not initialized. Must provide reference position to start().");
        }
    }

    void Moe::SmoothReferenceTrajectory::start(std::vector<double> ref_pos, std::vector<double> current_pos, Time current_time) {
        m_started = true;
        prev_ref_ = current_pos;
        ref_ = ref_pos;
        start_time_ = current_time;
    }

    void Moe::SmoothReferenceTrajectory::set_ref(std::vector<double> ref_pos, Time current_time) {
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

    double Moe::SmoothReferenceTrajectory::calculate_smooth_ref(std::size_t dof, Time current_time) {
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

    bool Moe::SmoothReferenceTrajectory::is_reached(std::vector<double> current_position, std::vector<double> tolerance){
        std::vector<char> check_dofs(n_dof,1);
        return check_goal_pos(ref_, current_position, check_dofs, tolerance, false);
    }

    void Moe::SmoothReferenceTrajectory::stop() {
        m_started = false;
    }    
    
    ///////////////////////// TORQUE SETTING FUNCTIONS /////////////////////////

    std::vector<double> Moe::set_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& ref, Time current_time) {
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
    std::vector<double> Moe::calc_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active) {
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
    std::vector<double> Moe::set_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active) {
        std::vector<double> command_torques = Moe::calc_pos_ctrl_torques(ref,active);
        // std::vector<double> command_torques(n_j,0.0);

        // if(std::count(active.begin(), active.end(), true) != ref.size()){
        //     LOG(Error) << "Size of 'ref' param must equal number of true values in 'active' param (default 4). Commanding 0 torques.";
        // }
        // else if(size(active) != 4){
        //     LOG(Error) << "Size of 'active' param must be 4. Commanding 0 torques.";
        // }
        // else{
        //     size_t active_counter = 0;
        //     for (std::size_t i = 0; i < n_j; ++i) {
        //         if (active[i]){
        //             command_torques[i] = joint_pd_controllers_[i].calculate(ref[active_counter], m_joint_positions[i], 0, m_joint_velocities[i]);
        //             active_counter++;
        //         }
        //     }
        // }
        set_raw_joint_torques(command_torques);

        return command_torques;
    }

    void Moe::set_raw_joint_torques(std::vector<double> new_torques) {
        
        // update reference
        m_joint_torques = new_torques;

        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            (*it)->set_torque(new_torques[it - moe_joints.begin()]);
        }
    }

    void Moe::set_high_gains(std::vector<bool> active){
        std::vector<double> high_gains = {2.0,3.0,3.0,3.0};
        for (std::size_t i=0; i<n_j; ++i) {
            if (active[i]){
                joint_pd_controllers_[i].kp = gains_P[i]*high_gains[i];
                joint_pd_controllers_[i].kd = gains_D[i]*1.0;
            }
        }
    }

    void Moe::set_normal_gains(std::vector<bool> active){
        for (std::size_t i=0; i<n_j; ++i) {
            if (active[i]){
                joint_pd_controllers_[i].kp = gains_P[i];
                joint_pd_controllers_[i].kd = gains_D[i];
            }
        }
    }

    /////////////////// GOAL CHECKING FUNCTIONS ///////////////////

    bool Moe::check_joint_goal_pos(std::vector<double> goal_pos, std::vector<char> check_dof, bool print_output) const {
        return check_goal_pos(goal_pos, get_joint_positions(), check_dof, m_goal_err_tol, print_output);
    }

    /////////////////// LIMIT CHECKING ON THE MEII ///////////////////

    bool Moe::any_limit_exceeded(){
        return (any_velocity_limit_exceeded() || any_torque_limit_exceeded());
    }

    bool Moe::any_velocity_limit_exceeded(){
        bool exceeded = false;
        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            if ((*it)->velocity_limit_exceeded())
                exceeded = true;
        }
        return exceeded;
    }

    bool Moe::any_torque_limit_exceeded(){
        bool exceeded = false;
        for (auto it = moe_joints.begin(); it != moe_joints.end(); ++it) {
            if ((*it)->torque_limit_exceeded())
                exceeded = true;
        }
        return exceeded;
    }

    //////////////// MISC USEFUL UTILITY FUNCTIONS ////////////////

    bool Moe::check_goal_pos(std::vector<double> goal_pos, std::vector<double> current_pos, std::vector<char> check_dof, std::vector<double> error_tol, bool print_output) {

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

    std::vector<double> Moe::copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec) {
        std::vector<double> std_vec(eigen_vec.size());
        for (int i = 0; i < eigen_vec.size(); ++i) {
            std_vec[i] = eigen_vec[i];
        }
        return std_vec;
    }

    Eigen::VectorXd Moe::copy_stdvec_to_eigvec(const std::vector<double>& std_vec) {
        Eigen::VectorXd eigen_vec(std_vec.size());
        for (size_t i = 0; i < std_vec.size(); ++i) {
            eigen_vec[i] = std_vec[i];
        }
        return eigen_vec;
    }

    void Moe::calibrate_auto(volatile std::atomic<bool>& stop){
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
            // daq_watchdog_kick();
            // check joint velocity limits
            if (any_velocity_limit_exceeded() || any_torque_limit_exceeded()) {
                stop = true;
                break;
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