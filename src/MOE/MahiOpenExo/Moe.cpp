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
        // enable DAQ
        daq_enable();
        std::vector<int32> encoder_offsets = { 0, 0, 0, 0};
        for (int i = 0; i < n_j; i++) {
            daq_encoder_write(i+1, encoder_offsets[i]);
        }
        daq_disable();
        stop = true;
    }

    void Moe::update(){
        for (size_t i = 0; i < n_j; i++){
            m_joint_positions[i] = moe_joints[i]->get_position();
            m_joint_velocities[i] = moe_joints[i]->get_velocity();
        }
    }

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

    std::vector<double> Moe::set_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active) {
        
        std::vector<double> command_torques(n_j,0.0);

        if(std::count(active.begin(), active.end(), true) != ref.size()){
            LOG(Error) << "Size of 'ref' param must equal number of true values in 'active' param (default 4). Commanding 0 torques.";
            return command_torques;
        }
        else if(size(active) != 4){
            LOG(Error) << "Size of 'active' param must be 4. Commanding 0 torques.";
            return command_torques;
        }

        for (std::size_t i = 0; i < n_j; ++i) {
            if (active[i]){
                command_torques[i] = joint_pd_controllers_[i].calculate(ref[i], m_joint_positions[i], 0, m_joint_velocities[i]);
            }
        }
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
        
    }
}