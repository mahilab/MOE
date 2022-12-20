#include <MOE/MOE.hpp>
#include <Mahi/Mpc.hpp>
#include <Mahi/Util.hpp>

using namespace casadi;
using namespace mahi::mpc;
using mahi::util::PI;

// format A matrix to only include the DOF being tested
SX format_A(SX A_full,std::vector<int> dof){
    SX A_res = SX::zeros(dof.size(),dof.size());
    for (auto i = 0; i < dof.size(); i++){
        for (auto j = 0; j < dof.size(); j++){
            A_res(i,j) = A_full(dof[i],dof[j]);
        }
    }
    return A_res;
}

// format B matrix to only include the DOF being tested
SX format_B(SX B_full,std::vector<int> dof){
    SX B_res = SX::zeros(dof.size(),1);
    for (auto i = 0; i < dof.size(); i++){
        B_res(i) = B_full(dof[i]);
    }
    return B_res;
}

int main(int argc, char* argv[])
{
    mahi::util::Options options("ex_mpc_gen_one_dof.exe", "Using MOE to generate MPC model with a selected set of states active");

    options.add_options()
        ("l,linear", "Generates linearized model.")
        ("d,dof",    "Vector of which DOFs are being tested. (-d 0,2)", cxxopts::value<std::vector<int>>())
        ("h,help",   "Prints help message.");
    
    auto result = options.parse(argc, argv);

    if (result.count("help")){
        std::cout << options.help() << std::endl;
        return 0;
    }

    bool linear = result.count("linear") > 0;

    std::vector<int> dof;
    if (result.count("dof")){
        dof = result["dof"].as<std::vector<int>>();
        sort(dof.begin(),dof.end());
    }
    else{
        LOG(mahi::util::Error) << "Must specify which DOF is being tested.";
        return -1;
    }

    // casadi variables for the parameters needed
    SX x, x_dot, u;
    std::string model_name;

    model_name = "moe";

    SX T0 = SX::sym("T0");
    SX T1 = SX::sym("T1");
    SX T2 = SX::sym("T2");
    SX T3 = SX::sym("T3");

    // create moe dynamic model
    moe::MoeDynamicModel moe_model;
    moe_model.set_user_params({3,  // forearm
                               4,  // counterweight position
                               0}); // shoulder rotation

    // can add an arm property file here if desired
    // moe_model.add_arm_props("C:/Users/nbd2/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Nathan6");

    // we start with the full set of pos and vel and reduce later based on active DOF
    SXVector q_full = {moe_model.q0,
                       moe_model.q1,
                       moe_model.q2,
                       moe_model.q3,
                       moe_model.q0_dot,
                       moe_model.q1_dot,
                       moe_model.q2_dot,
                       moe_model.q3_dot};
    SXVector q_vec;
    SXVector qd_vec;

    SXVector u_full = {T0, T1, T2, T3};
    SX u_full_SX = SX::vertcat(u_full);
    SXVector u_vec;

    // add all positions
    for (auto i = 0; i < dof.size(); i++) {
        q_vec.push_back(q_full[dof[i]]);
        qd_vec.push_back(q_full[dof[i]+4]);
        u_vec.push_back(u_full[dof[i]]);
    }

    // state vector
    SXVector x_vec = q_vec;
    x_vec.insert(x_vec.end(), qd_vec.begin(), qd_vec.end());
    x = SX::vertcat(x_vec);


    SX q_dot = SX::vertcat(qd_vec);

    // create a vector of what variables are being set to zero based on active dofs
    auto zero_variables_vec = q_full;
    for (int i = (dof.size()-1); i >= 0; i--) zero_variables_vec.erase(zero_variables_vec.begin()+dof[i]+4);
    for (int i = (dof.size()-1); i >= 0; i--) zero_variables_vec.erase(zero_variables_vec.begin()+dof[i]);
    casadi::SX zero_variables = vertcat(zero_variables_vec);

    // pulling relevant models from moe class
    auto G = moe_model.cas_get_G();
    auto V = moe_model.cas_get_V();
    auto Friction = moe_model.cas_get_Friction();
    auto B_full = u_full_SX - V - G - Friction;
    auto A_full = moe_model.cas_get_effective_M();

    // modify A and B to be correct size based on active DOF
    auto A_eom = format_A(A_full,dof);
    auto B_eom = format_B(B_full,dof);

    // solving eom
    SX q_d_dot_nonzero = solve(A_eom,B_eom);
    SX q_d_dot;

    // substitute constant variables into the EOM
    if (!zero_variables_vec.empty()){
        q_d_dot = substitute(q_d_dot_nonzero, zero_variables, std::vector<double>(8-dof.size()*2,0));
    }
    else {
        q_d_dot = q_d_dot_nonzero;
    }

    // finalize state and input into correct format
    u = SX::vertcat(u_vec);
    x_dot = vertcat(q_dot,q_d_dot);

    std::string dof_string = "";
    for (auto d : dof) dof_string += std::to_string(d);

    if (linear) model_name = "linear_" + model_name + "_j" + dof_string;
    
    // Bounds on state
    std::vector<double> x_min(x.size1(),-inf);
    std::vector<double> x_max(x.size1(), inf);

    // Bounds for control
    std::vector<double> u_min(u.size1(),-inf);
    std::vector<double> u_max(u.size1(), inf);

    // settings for multiple shooting constructions
    mahi::util::Time time_step  = mahi::util::milliseconds(linear ? 2 : 2);
    int num_shooting_nodes = 25;

    ModelParameters model_parameters(model_name, // name
                                     x.size1(),                // num_x
                                     u.size1(),                // num_u
                                     time_step,                // step_size
                                     num_shooting_nodes,       // num_shooting_nodes
                                     linear);                  // is_linear;                  

    // This is a helper class to actually generate the model
    ModelGenerator my_generator(model_parameters, x, x_dot, u);

    // these are the required methods to call to generate the model to use in c++
    my_generator.create_model();
    my_generator.generate_c_code();
    my_generator.compile_model();

    // just to check if it finishes
    std::cout << "Finished generating models" << std::endl;
    return 0;
}
