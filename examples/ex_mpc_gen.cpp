#include <MOE/MOE.hpp>
#include <Mahi/Mpc.hpp>
#include <Mahi/Util.hpp>

using namespace casadi;
using namespace mahi::mpc;
using mahi::util::PI;

int main(int argc, char* argv[])
{
    mahi::util::Options options("options.exe", "Simple Program Demonstrating Options");

    options.add_options()
        ("l,linear", "Generates linearized model.")
        ("h,help", "Prints help message.");
    
    auto result = options.parse(argc, argv);

    if (result.count("help")){
        std::cout << options.help() << std::endl;
        return 0;
    }

    bool linear = result.count("linear") > 0;

    SX x, x_dot, u;
    std::string model_name;

    model_name = "moe";
    
    SX T0 = SX::sym("T0");
    SX T1 = SX::sym("T1");
    SX T2 = SX::sym("T2");
    SX T3 = SX::sym("T3");
    moe::MoeDynamicModel moe_model;
    moe_model.set_user_params({3,  // forearm
                               4,  // counterweight position
                               0}); // shoulder rotation
    moe_model.add_arm_props("C:/Users/nbd2/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Nathan6");
    // state vector
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

    auto G = moe_model.cas_get_G();
    auto V = moe_model.cas_get_V();
    auto Friction = moe_model.cas_get_Friction();
    auto B_eom = u - V - G - Friction;
    auto A_eom = moe_model.cas_get_effective_M();
    SX q_d_dot = solve(A_eom,B_eom);
    x_dot = vertcat(q_dot,q_d_dot);

    if (linear) model_name = "linear_" + model_name;
    
    // Bounds on state
    std::vector<double> x_min(x.size1(),-inf);
    std::vector<double> x_max(x.size1(), inf);

    // Bounds for control
    std::vector<double> u_min(u.size1(),-inf);
    std::vector<double> u_max(u.size1(), inf);

    // settings for multiple shooting constructions
    mahi::util::Time time_step  = mahi::util::milliseconds(linear ? 2 : 2);
    int num_shooting_nodes = linear ? 25 : 25;
    // 4, 15 kind of worked
    ModelParameters model_parameters(model_name, // name
                                     x.size1(),                // num_x
                                     u.size1(),                // num_u
                                     time_step,                // step_size
                                     num_shooting_nodes,       // num_shooting_nodes
                                     linear);                  // is_linear;                  

    // 
    ModelGenerator my_generator(model_parameters, x, x_dot, u);

    my_generator.create_model();
    my_generator.generate_c_code();
    my_generator.compile_model();

    std::cout << "Finished generating models" << std::endl;
    return 0;
}
