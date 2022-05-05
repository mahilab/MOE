#include <MOE/MOE.hpp>
#include <Mahi/Casadi/ModelGenerator.hpp>
#include <Mahi/Util.hpp>

using namespace casadi;
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
    // SX q0 = SX::sym("q0");
    // SX q1 = SX::sym("q1");
    // SX q2 = SX::sym("q2");
    // SX q3 = SX::sym("q3");
    // SX q0_dot = SX::sym("q0_dot");
    // SX q1_dot = SX::sym("q1_dot");
    // SX q2_dot = SX::sym("q2_dot");
    // SX q3_dot = SX::sym("q3_dot");
    
    SX T0 = SX::sym("T0");
    SX T1 = SX::sym("T1");
    SX T2 = SX::sym("T2");
    SX T3 = SX::sym("T3");
    moe::MoeDynamicModel moe_model;
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

    // T_hardstop = SX::vertcat({hardstop_torque(q0,q0_dot,qmin,qmax,Khard,Bhard),
    //                           hardstop_torque(q1,q1_dot,qmin,qmax,Khard,Bhard),
    //                           hardstop_torque(q2,q2_dot,qmin,qmax,Khard,Bhard),
    //                           hardstop_torque(q3,q3_dot,qmin,qmax,Khard,Bhard)});



    SX q_dot = SX::vertcat({moe_model.q0_dot,
                            moe_model.q1_dot,
                            moe_model.q2_dot,
                            moe_model.q3_dot});

    // std::vector<double>  B_coef = {0.1215, 0.0252, 0.0019, 0.0029};
    // std::vector<double> Fk_coef = {   0.5, 0.1891, 0.0541, 0.1339};
    // std::vector<double>  B_coef = {0.0393, 0.0691, 0.0068, 0.0025};
    // std::vector<double> Fk_coef = {0.1838, 0.1572, 0.0996, 0.1685};

    // SX B = SX::vertcat({B_coef[0]*q0_dot*1.0, B_coef[1]*q1_dot*1.0, B_coef[2]*q2_dot*1.0, B_coef[3]*q3_dot*1.0});
    // SX Fk = SX::vertcat({Fk_coef[0]*tanh(q0_dot*10.0), Fk_coef[1]*tanh(q1_dot*10.0), Fk_coef[2]*tanh(q2_dot*10.0), Fk_coef[3]*tanh(q3_dot*10.0)});

        // Transmission Rations [rad/m]
    // std::vector<double> eta = {0.4200/4.50, 0.4706/8.75, 0.4735/9.00, 0.2210/6.00};
    // // Motor rotor inertias [Kg*m^2]
    // std::vector<double> Jm_ = {1340e-7, 137e-7, 137e-7, 34.7e-7};

    // SX Jm = SX::eye(4)*SX::vertcat({Jm_[0]/eta[0]/eta[0], Jm_[1]/eta[1]/eta[1], Jm_[2]/eta[2]/eta[2], Jm_[3]/eta[3]/eta[3]});;

    std::cout << "before G" << std::endl;
    auto G = moe_model.cas_get_G();
    std::cout << "after G" << std::endl;
    auto V = moe_model.cas_get_V();
    std::cout << "after V" << std::endl;
    auto Friction = moe_model.cas_get_Friction();
    std::cout << "after friction" << std::endl;
    auto B_eom = u - V - G - Friction;
    std::cout << "after B" << std::endl;
    auto A_eom = moe_model.cas_get_effective_M();
    std::cout << "after A" << std::endl;
    SX q_d_dot = solve(A_eom,B_eom);
    std::cout << A_eom.size() << std::endl;
    std::cout << B_eom.size() << std::endl;
    x_dot = vertcat(q_dot,q_d_dot);
    std::cout << "after V" << std::endl;

    if (linear) model_name = "linear_" + model_name;
    
    // Bounds on state
    std::vector<double> x_min(x.size1(),-inf);
    std::vector<double> x_max(x.size1(), inf);

    // Bounds for control
    std::vector<double> u_min(u.size1(),-inf);
    std::vector<double> u_max(u.size1(), inf);

    // settings for multiple shooting constructions
    mahi::util::Time time_step  = mahi::util::milliseconds(linear ? 2 : 10);
    int num_shooting_nodes = 25;

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
