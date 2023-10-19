#pragma once

#include <MOE/MahiOpenExo/MoeParameters.hpp>
#include<MOE/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <MOE/MahiOpenExo/Joint.hpp>
#include <Mahi/Robo/Control/PdController.hpp>
#include <Mahi/Util/Timing/Time.hpp>
#include <Mahi/Util/Device.hpp>
#include <array>
#include <vector>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/StdVector>

namespace moe {
    /// Class for controlling the Mahi Open Exo
    class MahiOpenExo : public mahi::util::Device{

    ///////////////////////// STANDARD CLASS FUNCTIONS AND PARAMS /////////////////////////

    public:
        /// Constructor
        MahiOpenExo(MoeParameters parameters = MoeParameters());
        /// Destructor
        ~MahiOpenExo();
        /// returns a pointer to robot joint [i]
        Joint* operator[](size_t joint_number){return moe_joints[joint_number].get();}
        /// Manually zero the encoders
        void calibrate(volatile std::atomic<bool>& stop_flag);
        /// Automatically zero the encoders
        void calibrate_auto(volatile std::atomic<bool>& stop_flag);
        /// Disables the robot and stops all smooth reference trajectories
        bool on_disable() override;
        /// Enables each of the joints on the MOE
        bool on_enable() override;

        double noise_degradation();

        double noise_lowFreq();

        std::vector<std::shared_ptr<Joint>> moe_joints; // vector of shared pointer of moe joints
        const MoeParameters params_;                    // parameters used to control the moe

        std::string name_; // name of the MOE based on the device
        static const std::size_t n_j = 4; // number of joints

    ///////////////////////// Mass Properties and Model Calculations /////////////////////////
    
    public:    
        MoeDynamicModel moe_dynamic_model; // class for the dynamic model of MOE 
        void set_user_parameters(UserParams newParams); // Sets the private struct user_params
        void set_user_parameters_from_json(std::string json_path); // Sets the private struct user_params from a json file
        UserParams get_subject_parameters(); // gets the private struct sub_params
        std::vector<double> calc_grav_torques(); // calculates the gravity vector based on mass props and current joint angles
    
    ///////////////////////// SMOOTH REFERENCE TRAJECTORY CLASS AND INSTANCES /////////////////////////

    public:
        /// Class that generates smooth reference trajectories that can be updated in real time
        /// This class probably isn't necessary for the MOE
        class SmoothReferenceTrajectory {

        public:
            /// Default Constructor
            SmoothReferenceTrajectory() {};
            /// Constructor with speed of joints, goal reference position, and specification of active DOFs
            SmoothReferenceTrajectory(std::vector<double> speed, std::vector<double> ref_pos, std::vector<bool> active_dofs = {true, true, true, true});

            /// starts a trajectory given the current position, and current time
            void start(std::vector<double> current_pos, mahi::util::Time current_time);
            /// starts a trajectory given the current position, current time, and sets a new reference position
            void start(std::vector<double> ref_pos, std::vector<double> current_pos, mahi::util::Time current_time);
            /// sets a new reference position, sends the current time, and restarts the trajectory
            void set_ref(std::vector<double> ref_pos, mahi::util::Time current_time);
            /// calculate the new reference by interpolating from the last trajectory at the given spped
            double calculate_smooth_ref(std::size_t dof, mahi::util::Time current_time);
            /// returns whether the reference is reached
            bool is_reached(std::vector<double> current_position, std::vector<double> tolerance);
            /// stops the reference trajectory
            void stop();
            /// returns whether or not the trajectory has started
            bool is_started() { return m_started; };

            size_t n_dof; // number of degrees of freedom in the smooth trajectory
            std::vector<bool> m_active_dofs; // list of the active degrees of freedom. should be of size 4
        private:
            bool m_is_valid; //
            mahi::util::Time start_time_ = mahi::util::seconds(0.0);
            std::vector<double> speed_;
            bool m_started = false;
            std::vector<double> ref_;
            bool m_ref_init = false;
            std::vector<double> prev_ref_;   
        };

        const std::vector<double> joint_speed = {0.25, 0.25, 0.15, 0.15}; // [rad/s] constant spped at which joint reference trajectories are interpolated

    ///////////////////////// TORQUE SETTING FUNCTIONS /////////////////////////
    // Use smooth_pos_ctrl_torques when you have a smooth reference trajectory
    // as the input. Use pos_ctrl_torques when you have a position reference to
    // use. Note that this ref should be close to the current position as to not 
    // cause a big jump in torque output. Use raw_joint_torques to provide the 
    // raw joint torques.

    public:
        /// sets the joint torques based on a smooth reference trajectory given to the function
        std::vector<double> set_smooth_pos_ctrl_torques(SmoothReferenceTrajectory& ref, mahi::util::Time current_time);
        /// sets the joint torques based on a reference given to the function
        std::vector<double> set_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active = std::vector<bool>(n_j,true));
        /// sets the PD control gains to a high value (5x)
        void set_high_gains(std::vector<bool> active = std::vector<bool>(n_j,true));
        /// sets the PD control gains to their normal value
        void set_normal_gains(std::vector<bool> active = std::vector<bool>(n_j,true));
        /// sets the joint torques to the input torque
        void set_raw_joint_torques(std::vector<double> new_torques);
        /// calculates the joint torques based on a reference given to the function (PD)
        std::vector<double> calc_pos_ctrl_torques(std::vector<double> ref, std::vector<bool> active = std::vector<bool>(n_j,true));
        

    /////////////////// GOAL CHECKING FUNCTIONS ///////////////////

    public:
        // compares the current position with a specified goal position for specified DOFs
        // Rename to check_joint_goal_pos
        bool check_joint_goal_pos(std::vector<double> goal_pos, std::vector<char> check_dof, bool print_output = false) const;
    
    private:
        std::vector<double> m_goal_err_tol = { 2.0 * mahi::util::DEG2RAD, 2.0 * mahi::util::DEG2RAD, 2.0 * mahi::util::DEG2RAD , 2.0 * mahi::util::DEG2RAD }; // [rad] tolerance for joint errors

    /////////////////// LIMIT CHECKING ON THE MOE ///////////////////

    public:
        /// loops through joints and checks if any of them have exceeded their velocity or torque limit
        bool any_limit_exceeded();
        /// loops through joints and checks if any of them have exceeded their velocity limit
        bool any_velocity_limit_exceeded();
        /// loops through joints and checks if any of them have exceeded their torque limit
        bool any_torque_limit_exceeded();

    /////////////////// PUBLIC FACING ROBOT STATE ACCESS ///////////////////

    public:
        void update();
        /// get vector of all joint positions
        std::vector<double> get_joint_positions() const { return m_joint_positions;};
        /// get single joint position
        double get_joint_position(std::size_t index) const {return m_joint_positions[index];};
        /// get vector of all joint velocities
        std::vector<double> get_joint_velocities() const {return m_joint_velocities;};
        /// get single joint velocity
        double get_joint_velocity(std::size_t index) const {return m_joint_velocities[index];};
        /// read all commanded joint torques (in joint space) from the desired joint. NOTE THAT THESE ARE COMMANDED TORQUE, SO IT IS NOT YET CLAMPED
        std::vector<double> get_joint_command_torques() const {return m_joint_torques;};
        /// return the commanded joint torque (in joint space) from the desired joint. NOTE THAT THESE ARE COMMANDED TORQUE, SO IT IS NOT YET CLAMPED
        double get_joint_command_torque(std::size_t index) const {return m_joint_torques[index];};

    private:
        std::vector<double> m_joint_positions;
        std::vector<double> m_joint_velocities;
        std::vector<double> m_joint_torques;

    /////////////////// ROBOT PD CONTROLLERS ///////////////////

    public:
        ////////////////////////////   Elbow     forearm PS      wrist FE         wrist RU
        std::vector<double> gains_P = {125.0,          25.0,         20.0,            20.0};
        std::vector<double> gains_D = { 1.75,          1.15,          1.0,            0.25};
        // double elbow_P = 125.0; // from common.hpp from testing
        // double elbow_D = 1.75; // from common.hpp from testing
        // double forearm_P = 25.0; // from common.hpp from testing
        // double forearm_D = 1.15; // from common.hpp from testing
        // double wrist_FE_P = 20.0; // from common.hpp from testing
        // double wrist_FE_D = 1.0; // from common.hpp from testing
        // double wrist_RU_P = 20.0; // from common.hpp from testing
        // double wrist_RU_D = 0.25; // from common.hpp from testing

        std::array<mahi::robo::PdController, n_j> joint_pd_controllers_ = {
            {
                mahi::robo::PdController(gains_P[0],gains_D[0]),
                mahi::robo::PdController(gains_P[1],gains_D[1]),
                mahi::robo::PdController(gains_P[2],gains_D[2]),
                mahi::robo::PdController(gains_P[3],gains_D[3])
            }
        };


    //////////////// MISC USEFUL UTILITY FUNCTIONS ////////////////
    
    private:
        /// compare a goal position with the current position for given joints and tolerances
        static bool check_goal_pos(std::vector<double> goal_pos, std::vector<double> current_pos, std::vector<char> check_joint, std::vector<double> error_tol, bool print_output = false);
        /// converts an eigen vector to a std vector
        std::vector<double> copy_eigvec_to_stdvec(const Eigen::VectorXd& eigen_vec);
        /// converts a std vector to an eigen vector
        Eigen::VectorXd copy_stdvec_to_eigvec(const std::vector<double>& std_vec);

    //////////////// PURE VIRTUAL FUNCTIONS FOR DERIVED CLASSES ////////////////
    public:
        /// enables the daq
        virtual bool daq_enable()=0;
        /// disables the daq
        virtual bool daq_disable()=0;
        /// opens the daq
        virtual bool daq_open()=0;
        /// closes the daq
        virtual bool daq_close()=0;
        /// starts the watchdog on the daq
        virtual bool daq_watchdog_start()=0;
        /// kicks the watchdog on the daq
        virtual bool daq_watchdog_kick()=0;
        /// reads all from the daq
        virtual bool daq_read_all()=0;
        /// writes all from the daq
        virtual bool daq_write_all()=0;
        /// sets encoders to input position (in counts)
        virtual bool daq_encoder_write(int index, mahi::util::int32 encoder_offset)=0;
    };
} // namespace moe