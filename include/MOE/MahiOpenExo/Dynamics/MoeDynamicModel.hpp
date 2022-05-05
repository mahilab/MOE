#pragma once
#include <MOE/MahiOpenExo/Dynamics/MoeMassProperties.hpp>
#include <MOE/MahiOpenExo/Dynamics/UserParams.hpp>
#include <Eigen/Dense>
#include <array>
#include <vector>

#ifdef MAHI_MPC
#include <casadi/casadi.hpp>
#endif

namespace moe {
    /// Class for for the Dynamic Model of MOE
    class MoeDynamicModel {
        public:
        /// Constructor
        MoeDynamicModel();
        /// Destructor
        ~MoeDynamicModel();

        /// Member variables for state
        private:
        // Number of joints
        int n_j = 4;
        std::vector<double> q = {0.0,0.0,0.0,0.0};
        std::vector<double> qd = {0.0,0.0,0.0,0.0};
        /// Struct containing the mass properties MOE
        MoeMassProperties moe_mass_props;
        /// Stuct containing the user params
        UserParams user_params;

        public:
        // Function to update the state
        void update(std::vector<double> joint_positions, std::vector<double> joint_velocities);
        // Function to update the J0 mass properties
        void update_J0();
        // Function to set the user parameters
        void set_user_params(UserParams newParams);
        // Function to get the user parameters
        UserParams get_user_params();

        private:
        double q_s = 0.0;
        double dist = 0.28024875;
        double g = 9.81;

        // Friction Parameters
        std::vector<double> Fk_coeff = {0.1838, 0.1572, 0.0996, 0.1685};
        std::vector<double> B_coeff = {0.0393, 0.0691, 0.0068, 0.0025};
        // Rotor Inertias
        std::vector<double> rotor_inertias = {1340e-7, 137e-7, 137e-7, 34.7e-7};
        std::vector<double> joint_etas = {0.4200/4.50, 0.4706/8.75, 0.4735/9.00, 0.2210/6.00};


        // Functions for dynamic equations
        public:
        // Function to get M
        Eigen::MatrixXd get_M();
        // Function to get V
        Eigen::VectorXd get_V();
        // Function to get G
        Eigen::VectorXd get_G();
        // Function to get Friction
        Eigen::VectorXd get_Friction();
        // Function to get the rotor inertia
        Eigen::MatrixXd get_rotor_inertia();
        // Function to get effective mass
        Eigen::MatrixXd get_effective_M();

#ifdef MAHI_MPC
        public:
        // Function to get M
        casadi::SX cas_get_M();
        // Function to get V
        casadi::SX cas_get_V();
        // Function to get G
        casadi::SX cas_get_G();
        // Function to get Friction
        casadi::SX cas_get_Friction();
        // Function to get the rotor inertia
        casadi::SX cas_get_rotor_inertia();
        // Function to get effective mass
        casadi::SX cas_get_effective_M();

        casadi::SX q0;
        casadi::SX q1;
        casadi::SX q2;
        casadi::SX q3;
        casadi::SX q0_dot;
        casadi::SX q1_dot;
        casadi::SX q2_dot;
        casadi::SX q3_dot;
        
        casadi::SX cas_q;
        casadi::SX cas_qd;
#endif

    };
}