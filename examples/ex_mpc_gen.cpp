#include <MOE/MOE.hpp>
#include <Mahi/Mpc.hpp>
#include <Mahi/Util.hpp>

using namespace casadi;
using namespace mahi::mpc;
using mahi::util::PI;

int main(int argc, char* argv[])
{
    mahi::util::Options options("ex_mpc_gen.exe", "Using MOE to generate MPC model with all states active");

    options.add_options()
        ("l,linear", "Generates linearized model.")
        ("h,help", "Prints help message.");
    
    auto result = options.parse(argc, argv);

    if (result.count("help")){
        std::cout << options.help() << std::endl;
        return 0;
    }

    bool linear = result.count("linear") > 0;

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

    // state vector
    // q0, q1, ... already exist in moe, so this is just pulling them into a single vector
    x = SX::vertcat({moe_model.q0,
                     moe_model.q1,
                     moe_model.q2,
                     moe_model.q3,
                     moe_model.q0_dot,
                     moe_model.q1_dot,
                     moe_model.q2_dot,
                     moe_model.q3_dot});
    u = SX::vertcat({T0,T1,T2,T3});

    SX q_dot = SX::vertcat({moe_model.q0_dot,
                            moe_model.q1_dot,
                            moe_model.q2_dot,
                            moe_model.q3_dot});

    // pulling relevant models from moe class
    auto G = moe_model.cas_get_G();
    auto V = moe_model.cas_get_V();
    auto Friction = moe_model.cas_get_Friction();
    auto B_eom = u - V - G - Friction;
    auto A_eom = moe_model.cas_get_effective_M();
    SX q_d_dot = solve(A_eom,B_eom);
    x_dot = vertcat(q_dot,q_d_dot);

    // I normally do linear because it is a lot faster and doesn't really affect performance. I would use that by default
    if (linear) model_name = "linear_" + model_name;
    
    // Bounds on state
    std::vector<double> x_min(x.size1(),-inf);
    std::vector<double> x_max(x.size1(), inf);

    // Bounds for control
    std::vector<double> u_min(u.size1(),-inf);
    std::vector<double> u_max(u.size1(), inf);

    // settings for multiple shooting constructions
    mahi::util::Time time_step  = mahi::util::milliseconds(linear ? 2 : 2); // time step for the MPC formulation
    int num_shooting_nodes = linear ? 25 : 25; // number of time steps into the future we look at

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
