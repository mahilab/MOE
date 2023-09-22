#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
namespace moe {

    // Constructor
    // If MAHI_MPC is set to true, create the casadi state variables
    MoeDynamicModel::MoeDynamicModel()
     {
#ifdef MAHI_MPC
        q0 = casadi::SX::sym("q0");
        q1 = casadi::SX::sym("q1");
        q2 = casadi::SX::sym("q2");
        q3 = casadi::SX::sym("q3");
        q0_dot = casadi::SX::sym("q0_dot");
        q1_dot = casadi::SX::sym("q1_dot");
        q2_dot = casadi::SX::sym("q2_dot");
        q3_dot = casadi::SX::sym("q3_dot");
        cas_q = casadi::SX::vertcat({q0,q1,q2,q3});
        cas_qd = casadi::SX::vertcat({q0_dot,q1_dot,q2_dot,q3_dot});

        q0m = casadi::MX::sym("q0m");
        q1m = casadi::MX::sym("q1m");
        q2m = casadi::MX::sym("q2m");
        q3m = casadi::MX::sym("q3m");
        q0m_dot = casadi::MX::sym("q0m_dot");
        q1m_dot = casadi::MX::sym("q1m_dot");
        q2m_dot = casadi::MX::sym("q2m_dot");
        q3m_dot = casadi::MX::sym("q3m_dot");
        cas_qm = casadi::MX::vertcat({q0m,q1m,q2m,q3m});
        cas_qdm = casadi::MX::vertcat({q0m_dot,q1m_dot,q2m_dot,q3m_dot});
#endif
    }
    // Destructor
    MoeDynamicModel::~MoeDynamicModel() {

    }

    //  Updates the state of MoeDynamicModel with position and velocity
    void MoeDynamicModel::update(std::vector<double> joint_positions, std::vector<double> joint_velocities) {
        q = joint_positions;
        qd = joint_velocities;
    }

    // Updates the mass props of the elbow joint based on the user params
    void MoeDynamicModel::update_J0() {
        // If the arm is initialized, cannot update J0
        if(!is_arm_initialized){
            // First time this is called, save the value of the slider and counterweight COM
            static double original_slider_pcy = moe_mass_props.J0_slider.Pcy;
            static double original_counterweight_pcy = moe_mass_props.J0_counterweight.Pcy;
            // Recalculate COM based on user Params
            moe_mass_props.J0_slider.Pcy = original_slider_pcy + (user_params.forearm_location-3)*0.005;
            moe_mass_props.J0_counterweight.Pcy = original_counterweight_pcy + (user_params.cw_location-4)*0.0127;
            // Create vector of the bodies to combine
            std::vector<JointProperties> bodies = {moe_mass_props.J0_main,moe_mass_props.J0_counterweight,moe_mass_props.J0_slider};
            // Combine mass properties and assign to J0
            moe_mass_props.J0 = combine_bodies(bodies);
            // Set warning for re-initializing
            if (is_J0_initialized) {
                LOG(mahi::util::Warning) << "J0 is already initialized. you are overwriting previous mass properties.";
            }
            // update initialization boolean
            is_J0_initialized = true;
        }
        else{
            LOG(mahi::util::Error) << "Cannot reinitialize J0 after adding arm properties.";
        }
    }

    // Gives access to setting the user params based on the current set up
    void MoeDynamicModel::set_user_params(UserParams newParams) {
        if(newParams.shoulder_location > 12) {
            LOG(mahi::util::Error) << "Shoulder location greater than 12. Value should be number of notches away from 0 degrees. Not setting parameters.";
            return;
        }
        if(newParams.forearm_location < 3 || newParams.forearm_location > 11) {
            LOG(mahi::util::Error) << "Forearm location not within bounds. Not setting parameters.";
            return;
        }
        if(newParams.cw_location < 1 || newParams.cw_location > 13) {
            LOG(mahi::util::Error) << "Counterweight location not within bounds. Not setting parameters.";
            return;
        }
        user_params = newParams;
        q_s = mahi::util::DEG2RAD*user_params.shoulder_location*15.0; // calculate shoulder angle
        dist = 0.28024875 - 0.005*(user_params.forearm_location - 3); // calculate distance between elbow frame and wrist frames
        // Every time user params are set, update the elbow joint mass props
        update_J0();
    }

    // Function to set user parameters directly from json
    void MoeDynamicModel::set_user_params_from_json(std::string json_path) {
        UserParams newParams = UserParams::get_from_json(json_path);
        set_user_params(newParams);
    }

    // Function to allow access to the current user_params
    UserParams MoeDynamicModel::get_user_params() {
        return user_params;
    }

    // Calculates the a 4x4 diagonal matrix of the rotor interias
    // This allows for the effective mass to be calculated as M + rotor_inertia
    Eigen::MatrixXd MoeDynamicModel::get_rotor_inertia() {
        Eigen::VectorXd rotor_inertia_vector = Eigen::VectorXd::Zero(n_j);

        for (int i = 0; i < rotor_inertia_vector.size(); i++) {
            rotor_inertia_vector(i) = rotor_inertias[i]/joint_etas[i]/joint_etas[i];
        }
        // Transform vector into diagonal matrix
        // Turns element-wise multiplication into matrix multiplication
        Eigen::MatrixXd rotor_inertia = rotor_inertia_vector.asDiagonal();

        return rotor_inertia;
    }

    // Finds the effective mass using the internal state of MoeDynamicModel
    // effective_M*qddot = M*qddot + rotor_inertia_vector.*qddot (in matlab)
    Eigen::MatrixXd MoeDynamicModel::get_effective_M() {
        // get_rotor_inertia() is a diagonal matrix
        Eigen::MatrixXd effective_M = get_M() + get_rotor_inertia();
        return effective_M;
    }

    // This function will take in a vector of JointProperties and combine their mass properties
    JointProperties MoeDynamicModel::combine_bodies(std::vector<JointProperties> bodies) {
        // determine size of the input vector
        auto num_bodies = bodies.size();
        // intialize the combine body mass props to 0
        JointProperties combined_body{0,0,0,0,0,0,0,0,0,0};
        // Loop through to add the COMs and masses
        for (int i = 0; i < num_bodies; i++) {
            combined_body.Pcx += bodies[i].Pcx*bodies[i].m;
            combined_body.Pcy += bodies[i].Pcy*bodies[i].m;
            combined_body.Pcz += bodies[i].Pcz*bodies[i].m;
            combined_body.m += bodies[i].m;
        }
        // Divide each COM by total mass (because above loop just has addition)
        combined_body.Pcx = combined_body.Pcx/combined_body.m;
        combined_body.Pcy = combined_body.Pcy/combined_body.m;
        combined_body.Pcz = combined_body.Pcz/combined_body.m;
        // Loop through using parallel axis thm to find inertias using prior calcs
        for (int i = 0; i < num_bodies; i++) {
            combined_body.Icxx += bodies[i].Icxx + bodies[i].m*((bodies[i].Pcy-combined_body.Pcy)*(bodies[i].Pcy-combined_body.Pcy) + (bodies[i].Pcz-combined_body.Pcz)*(bodies[i].Pcz-combined_body.Pcz));
            combined_body.Icyy += bodies[i].Icyy + bodies[i].m*((bodies[i].Pcx-combined_body.Pcx)*(bodies[i].Pcx-combined_body.Pcx) + (bodies[i].Pcz-combined_body.Pcz)*(bodies[i].Pcz-combined_body.Pcz));
            combined_body.Iczz += bodies[i].Iczz + bodies[i].m*((bodies[i].Pcx-combined_body.Pcx)*(bodies[i].Pcx-combined_body.Pcx) + (bodies[i].Pcy-combined_body.Pcy)*(bodies[i].Pcy-combined_body.Pcy));
            combined_body.Icxy += bodies[i].Icxy + bodies[i].m*((bodies[i].Pcx-combined_body.Pcx)*(bodies[i].Pcy-combined_body.Pcy));
            combined_body.Icxz += bodies[i].Icxz + bodies[i].m*((bodies[i].Pcx-combined_body.Pcx)*(bodies[i].Pcz-combined_body.Pcz));
            combined_body.Icyz += bodies[i].Icyz + bodies[i].m*((bodies[i].Pcy-combined_body.Pcy)*(bodies[i].Pcz-combined_body.Pcz));
        }
        
        return combined_body;
    }

    // Function that reads in jsons containing subject arm properties
    // This function will read in the jsons named in the format shown and update the moe_mass_props struct
    // Format: folder_path/J0_arm_mass_props.json
    //         folder_path/J1_arm_mass_props.json
    //         folder_path/J2_arm_mass_props.json
    //         folder_path/J3_arm_mass_props.json
    //         folder_path/armFriction.json (optional)
    void MoeDynamicModel::add_arm_props(std::string folder_path, bool friction_exists) {
        if (is_arm_initialized == false) {
            // File path for each json
            std::string efe_file_path = folder_path + "/J0_arm_mass_properties.json";
            std::string fps_file_path = folder_path + "/J1_arm_mass_properties.json";
            std::string wfe_file_path = folder_path + "/J2_arm_mass_properties.json";
            std::string wru_file_path = folder_path + "/J3_arm_mass_properties.json";
            std::string friction_file_path = folder_path + "/armFriction.json";
            // Make a vector of all the arm properties
            std::vector<JointProperties> arm_props = {JointProperties::get_from_json(efe_file_path),
                                                      JointProperties::get_from_json(fps_file_path),
                                                      JointProperties::get_from_json(wfe_file_path),
                                                      JointProperties::get_from_json(wru_file_path)};
            // Combine the arm properties to moe_mass_props
            moe_mass_props.J0 = combine_bodies({moe_mass_props.J0,arm_props[0]});
            moe_mass_props.J1 = combine_bodies({moe_mass_props.J1,arm_props[1]});
            moe_mass_props.J2 = combine_bodies({moe_mass_props.J2,arm_props[2]});
            moe_mass_props.J3 = combine_bodies({moe_mass_props.J3,arm_props[3]});

            // get the arm friction properties and add them to the current properties
            if (friction_exists) {
                json friction_json;
                std::ifstream friction_file(friction_file_path);
                friction_file >> friction_json;
                auto Fk_arm = friction_json["Fk"].get<std::vector<double>>();
                auto B_arm = friction_json["B"].get<std::vector<double>>();
                for (int i = 0; i < 4; i++) {
                    Fk_coeff[i] += Fk_arm[i];
                    B_coeff[i] += B_arm[i];
                }
            }

            is_arm_initialized = true;
        } 
        // Unable to add arm properties more than once
        else {
           LOG(mahi::util::Error) << "Arm props are already initialized. Returning those values.";
        }
    }
// The same functions as above, but using casadi variables
#ifdef MAHI_MPC
    casadi::SX MoeDynamicModel::cas_get_rotor_inertia() {
        casadi::SX rotor_inertia_vector = casadi::SX::zeros(n_j,1);

        for (int i = 0; i < rotor_inertias.size(); i++) {
            rotor_inertia_vector(i,0) = rotor_inertias[i]/joint_etas[i]/joint_etas[i];
        }
        casadi::SX rotor_inertia = casadi::SX::diag(rotor_inertia_vector);

        return rotor_inertia;
    }

    casadi::SX MoeDynamicModel::cas_get_effective_M() {
        casadi::SX effective_M = cas_get_M() + cas_get_rotor_inertia();
       
       
        return effective_M;
    }

        casadi::MX MoeDynamicModel::cas_get_rotor_inertia_m() {
        casadi::MX rotor_inertia_vector = casadi::MX::zeros(n_j,1);

        for (int i = 0; i < rotor_inertias.size(); i++) {
            rotor_inertia_vector(i,0) = rotor_inertias[i]/joint_etas[i]/joint_etas[i];
        }
        casadi::MX rotor_inertia = casadi::MX::diag(rotor_inertia_vector);

        return rotor_inertia;
    }

    casadi::MX MoeDynamicModel::cas_get_effective_M_m() {
        casadi::MX effective_M = cas_get_M_m() + cas_get_rotor_inertia_m();
       
       
        return effective_M;
    }

#endif
}
