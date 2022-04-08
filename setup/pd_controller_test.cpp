#include<Mahi/Daq.hpp>
#include<Mahi/Util.hpp>
#include<Mahi/Robo.hpp>
#include<common.hpp>

using namespace mahi::daq;
using namespace mahi::util;
using namespace mahi::robo;

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
    // trajectory parameters -> desired_pos = traj_amplitude*sin(2.0*PI*traj_frequency*t.as_seconds()) + traj_offset
    double traj_frequency = 0.5; // Hz
    double traj_amplitude = 20 * DEG2RAD; // amplitude in radians
    double traj_offset = 0*DEG2RAD; // sinwave

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

    Time traj_time = 10_s;
    // END EXPERIMENT PARAMETERS

    // initialize q8
    for (auto i = 0; i < moe_n_dof; i++){
        // initialize DO
        q8.DO.enable_values[i]  = TTL_LOW;
        q8.DO.disable_values[i] = TTL_LOW;

        // initialize AO
        q8.AO.enable_values[i]  = 0;
        q8.AO.disable_values[i] = 0;
        
        // initialize encoders
        q8.encoder.units[encoder_channel[i]] = (2*PI/encoder_cprs[i])*gear_ratios[i];
        q8.encoder.zero(encoder_channel[i]);
    }

    PdController myController = PdController(Kps[active_joint],Kds[active_joint]);

    q8.enable();

    prompt("Press ENTER to start I/O loop.");
    
    int cycle_counter = 0;
    int cycles_between_print = 500;
    Time t = Time::Zero;
    Timer timer(1000_Hz);

    q8.DO[active_joint] = TTL_HIGH;

    while (!stop && t < traj_time) {
        q8.read_all();

        double desired_pos = traj_amplitude*sin(2.0*PI*traj_frequency*t.as_seconds()) + traj_offset;

        // get position and velocity of joint we are testing
        double current_pos = q8.encoder.positions[encoder_channel[active_joint]];
        double current_vel = q8.velocity.velocities[encoder_channel[active_joint]];

        // calc desired torque using pd controller (des vel is always 0)
        double desiredTorque = myController.calculate(desired_pos,current_pos,0.0,current_vel);
        
        // compute the voltage out
        double commandedCurrent = desiredTorque*gear_ratios[active_joint]/Kts[active_joint]; // Nm / (Nm / A) = [A]
        double out = (switch_dirs[active_joint] ? -1.0 : 1.0)*commandedCurrent/amp_ratios[active_joint]; // A / (A / V) = [V]
        q8.AO[active_joint] = out;
        q8.write_all();
        
        // print but only every {cycles_between_print} cycles so that we don't hog up all computation time
        if ((cycle_counter++ % cycles_between_print) == 0){
            print("Current Position: {:+.2f} deg. Desired Position {:+.2f} deg.",current_pos,desired_pos);
            print("Torque {:+.2f}",desiredTorque);
        }        

        // check if we have tripped any of our safety limits
        if (torque_limits_exceeded({desiredTorque},{torque_limits[active_joint]}) || 
            velocity_limits_exceeded({current_vel},{velocity_limits[active_joint]})){
            stop = true;
        }

        t = timer.wait();
    }

    q8.DO[active_joint] = TTL_LOW;

    q8.disable();
    q8.close();
    return 0;
}