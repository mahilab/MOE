#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
namespace moe {
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
#endif
    }

    MoeDynamicModel::~MoeDynamicModel() {

    }
    void MoeDynamicModel::update(std::vector<double> joint_positions, std::vector<double> joint_velocities) {
        q = joint_positions;
        qd = joint_velocities;
    }

    void MoeDynamicModel::update_J0() {
        if(!is_arm_initialized){
            static double original_slider_pcy = moe_mass_props.J0_slider.Pcy;
            static double original_counterweight_pcy = moe_mass_props.J0_counterweight.Pcy;

            moe_mass_props.J0_slider.Pcy = original_slider_pcy + (user_params.forearm_location-3)*0.005;
            moe_mass_props.J0_counterweight.Pcy = original_counterweight_pcy + (user_params.cw_location-4)*0.0127;
            std::vector<JointProperties> bodies = {moe_mass_props.J0_main,moe_mass_props.J0_counterweight,moe_mass_props.J0_slider};
            moe_mass_props.J0 = combine_bodies(bodies);

            if (is_J0_initialized) {
                LOG(mahi::util::Warning) << "J0 is already initialized. you are overwriting previous mass properties.";
            }
            is_J0_initialized = true;
        }
        else{
            LOG(mahi::util::Error) << "Cannot reinitialize J0 after adding arm properties.";
        }
    }

    void MoeDynamicModel::set_user_params(UserParams newParams) {
        user_params = newParams;
        q_s = mahi::util::DEG2RAD*user_params.shoulder_ang;
        dist = 0.28024875 - 0.005*(user_params.forearm_location - 3);
        update_J0();
    }

    UserParams MoeDynamicModel::get_user_params() {
        return user_params;
    }

    Eigen::MatrixXd MoeDynamicModel::get_rotor_inertia() {
        Eigen::VectorXd rotor_inertia_vector = Eigen::VectorXd::Zero(n_j);

        for (int i = 0; i < rotor_inertia_vector.size(); i++) {
            rotor_inertia_vector(i) = rotor_inertias[i]/joint_etas[i]/joint_etas[i];
        }
        Eigen::MatrixXd rotor_inertia = rotor_inertia_vector.asDiagonal();

        return rotor_inertia;
    }

    Eigen::MatrixXd MoeDynamicModel::get_effective_M() {
        Eigen::MatrixXd effective_M = get_M() + get_rotor_inertia();
        return effective_M;
    }

    JointProperties MoeDynamicModel::combine_bodies(std::vector<JointProperties> bodies) {
        auto num_bodies = bodies.size();
        JointProperties combined_body;
        for (int i = 0; i < num_bodies; i++) {
            combined_body.Pcx += bodies[i].Pcx*bodies[i].m;
            combined_body.Pcy += bodies[i].Pcy*bodies[i].m;
            combined_body.Pcz += bodies[i].Pcz*bodies[i].m;
            combined_body.m += bodies[i].m;
        }
        combined_body.Pcx = combined_body.Pcx/combined_body.m;
        combined_body.Pcy = combined_body.Pcy/combined_body.m;
        combined_body.Pcz = combined_body.Pcz/combined_body.m;

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

    void MoeDynamicModel::add_arm_props(std::string folder_path) {
        if (is_arm_initialized == false) {
            // File path for each json
            std::string efe_file_path = folder_path + "/J0_arm_mass_properties.json";
            std::string fps_file_path = folder_path + "/J1_arm_mass_properties.json";
            std::string wfe_file_path = folder_path + "/J2_arm_mass_properties.json";
            std::string wru_file_path = folder_path + "/J3_arm_mass_properties.json";
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
            is_arm_initialized = true;
        } else {
           LOG(mahi::util::Error) << "Arm props are already initialized. Returning those values.";
        }
    }

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
#endif
}
