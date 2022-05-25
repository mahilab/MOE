#include <MOE/MOE.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Daq.hpp>
#include <Mahi/Robo.hpp>
#include <vector>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;
using namespace mahi::com;
using namespace moe;

using mahi::robo::WayPoint;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char* argv[]) {
 // register ctrl-c handler
    register_ctrl_handler(handler);


    MoeDynamicModel moe_dynamic_model;

    moe_dynamic_model.set_user_params(UserParams{3,4,0});
    moe_dynamic_model.update({-0.61223, 0.000458711, 0.00315049, -0.00123106},{0,0,0,0});

    // moe_dynamic_model.update_J0();
    std::cout << "Gravity Torques:" << moe_dynamic_model.get_G() << std::endl;
    // std::cout << "Positions:" << moe_dynamic_model.q << std::endl;
    

    //JointProperties efe_joint_properties = JointProperties::get_from_json("C:/Users/jb139/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Nathan/J0_arm_mass_properties.json");
    // std::vector<JointProperties> arm_properties = {JointProperties::get_from_json("C:/Users/jb139/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Nathan/J0_arm_mass_properties.json")};
    // std::cout << arm_properties[0].m << std::endl;
    moe_dynamic_model.add_arm_props("C:/Users/jb139/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Nathan");
    std::cout << "New Gravity Torques:" << moe_dynamic_model.get_G() << std::endl;
    // moe_dynamic_model.add_arm_props("C:/Users/jb139/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Nathan");
    std::cout << "HERE" << std::endl;
    moe_dynamic_model.set_user_params_from_json("C:/Users/jb139/Box/MAHI_Open_Exo/Model_Optimization/ArmOpt/Jeffery2/userParams.json");
    // std::cout << moe_dynamic_model.user_params.cw_location << std::endl;
    return 0;
}
