#include<Mahi/Daq.hpp>
#include<Mahi/Util.hpp>
#include<common.hpp>

using namespace mahi::daq;
using namespace mahi::util;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[])
{
    register_ctrl_handler(handler);

    // make options
    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("j,joint_num", "joint number to test", cxxopts::value<int>())
		("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    // if -h, print the help option
    if (result.count("help") > 0) {
        print_var(options.help());
        return 0;
    }

    Q8Usb q8;
    if (!q8.is_open())
        return 1;
    
    // EXPERIMENT PARAMETERS
    int active_joint = 0;
    if (result.count("joint_num")){
        active_joint = result["joint_num"].as<int>();
    }
    else {
        print_var("Please specify a joint number");
        return -1;
    }

    if (active_joint < 0 || active_joint > 3) {
        print_var("Joint number must be between 0 and 3");
        return -1;
    }

    double torque_increment = 0.005; // [Nm];
    double max_torque = 0.1; // [Nm]
    //

    // initialize q8
    for (auto i = 0; i < moe_n_dof; i++){
        q8.DO.enable_values[i]  = TTL_LOW;
        q8.DO.disable_values[i] = TTL_LOW;
        q8.AO.enable_values[i]  = 0;
        q8.AO.disable_values[i] = 0;
    }
    
    q8.enable();
    sleep(1_ms);

    // don't do anything until the user hits enter
    prompt("Press ENTER to start I/O loop.");
    std::cout << "Press ctrl+c to exit at any time" << std::endl;
    
    // enable motor
    q8.DO[active_joint] = TTL_HIGH;
    
    // start torque at 0 Nm
    double desiredTorque = 0.0; // Nm
    Timer timer(50_ms);

    while (desiredTorque < max_torque && !stop){
        // calculate desired output voltage based on known variables
        double commandedCurrent = desiredTorque/Kts[active_joint]; // Nm / (Nm / A) = [Nm]
        double out = (switch_dirs[active_joint] ? -1.0 : 1.0)*commandedCurrent/amp_ratios[active_joint]; // A / (A / V) = [V]

        // write to daq
        q8.AO[active_joint] = out;
        q8.write_all();
        
        //output to command line to monitor results and check for consistency
        // print("Command Torque: {} Nm.\nCommand Current: {} A\nCommand Voltage: {} V",
                                                                        // desiredTorque,
                                                                        // commandedCurrent,
                                                                        // out);

        // monitor for enter keypress to increase torque
        int key_press = get_key_nb();
        if(key_press == KEY_ENTER){
            desiredTorque += torque_increment; // Nm
            //output to command line to monitor results and check for consistency
            print("Command Torque: {} Nm.\nCommand Current: {} A\nCommand Voltage: {} V",
                                                                        desiredTorque,
                                                                        commandedCurrent,
                                                                        out);
        }
        timer.wait();
    }

    // disable motor that was previously enabled
    q8.DO[active_joint] = TTL_LOW;

    q8.disable();
    q8.close();
    return 0;
}