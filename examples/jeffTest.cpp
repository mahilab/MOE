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

    // make options
    // Options options("grav_comp.exe","Gravity Comp Demo");
    // options.add_options()
	// 	("c,calibrate", "Calibrates the MAHI Exo-II")
    //     ("n,no_torque", "trajectories are generated, but not torque provided")
    //     ("v,virtual", "example is virtual and will communicate with the unity sim")
	// 	("h,help", "Prints this help message");

    // auto result = options.parse(argc, argv); 
    // // if -h, print the help option
    // if (result.count("help") > 0) {
    //     print_var(options.help());
    //     return 0;
    // }  

    // // enable Windows realtime
    // enable_realtime();

    // Time Ts = milliseconds(1);  // sample period for DAQ

    // std::shared_ptr<Moe> moe = nullptr;
    // std::shared_ptr<Q8Usb> daq = nullptr;

    // if(result.count("virtual") > 0){
    //     MoeConfigurationVirtual config_vr;
    //     moe = std::make_shared<MahiOpenExoVirtual>(config_vr);
    // }
    // else{
    //     daq = std::make_shared<Q8Usb>();
    //     daq->open();

    //     MoeConfigurationHardware config_hw(*daq,VelocityEstimator::Hardware);
    //     std::vector<TTL> idle_values(8,TTL_LOW);
    //     daq->DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
    //     daq->DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
    //     daq->DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);   

    //     moe = std::make_shared<MahiOpenExoHardware>(config_hw);
   
    // }

    MoeDynamicModel moe_dynamic_model;

    moe_dynamic_model.set_user_params(UserParams{3,4,0});
    moe_dynamic_model.update({-0.61223, 0.000458711, 0.00315049, -0.00123106},{0,0,0,0});

    // moe_dynamic_model.update_J0();
    std::cout << "Gravity Torques:" << moe_dynamic_model.get_G() << std::endl;
    // std::cout << "Positions:" << moe_dynamic_model.q << std::endl;



    
    return 0;
}
