#pragma once
#include <MOE/MahiOpenExo/Dynamics/MoeMassProperties.hpp>
#include <MOE/MahiOpenExo/Dynamics/UserParams.hpp>
#include <Eigen/Dense>
#include <array>
#include <vector>

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

        // Friction Parameters
        std::vector<double> Fk_coeff = {0.1838, 0.1572, 0.0996, 0.1685};
        std::vector<double> B_coeff = {0.0393, 0.0691, 0.0068, 0.0025};
        // Rotor Inertias
        std::vector<double> rotor_inertias = {0.0,0.0,0.0,0.0};


        // Functions for dynamic equations
        public:
        // Function to get M
        Eigen::MatrixXd get_M();
        // Function to get V
        Eigen::MatrixXd get_V();
        // Function to get G
        Eigen::VectorXd get_G();
        // Function to get Friction
        Eigen::VectorXd get_Friction();
        // Function to get the rotor inertia
        Eigen::MatrixXd get_rotor_inertia();
        // Function to get effective mass
        Eigen::MatrixXd get_effective_M();

    };
}