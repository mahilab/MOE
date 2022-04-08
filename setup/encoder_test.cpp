#include<Mahi/Daq.hpp>
#include<Mahi/Util.hpp>
#include<common.hpp>

using namespace mahi::daq;
using namespace mahi::util;

int main(int arc, char const *argv[])
{
    Q8Usb q8;
    if (!q8.is_open())
        return -1;

    std::vector<bool> active_joints = {true,   // joint 1 (elbow)
                                       true,  // joint 2 (forearm ps)
                                       true,  // joint 3 (wrist fe)
                                       true}; // joint 4 (wrist ru)

    for (auto i = 0; i < moe_n_dof; i++){
        q8.encoder.units[encoder_channel[i]] = (2.0*PI/encoder_cprs[i])*gear_ratios[i];
        q8.encoder.zero(encoder_channel[i]);
    }
    
    q8.enable();

    Timer timer(1_s);
    Time t = 0_s;
    auto test_time = 10_s;

    while (t < test_time){
        q8.read_all();
        for (auto i = 0; i < moe_n_dof; i++){
            if (active_joints[i]){
                print("encoder[{}]: {} counts = {:+.2f} rad. -> {:+2f} rad/s",
                            i,
                            q8.encoder[encoder_channel[i]],
                            q8.encoder.positions[encoder_channel[i]],
                            q8.velocity.velocities[encoder_channel[i]]);
            }
        }
        
        t = timer.wait();
    }

    q8.disable();
    q8.close();
    return 0;
}