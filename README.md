# MOE

This repository contains the source code used to run the MOE robot. Examples of applications used to run MOE can be found in the [examples folder](https://github.com/mahilab/MOE/tree/main/examples).

An example of a repository that can be used to run Model Predictive Control on MOE can be found [here](https://github.com/nathandunk/moe-mpc).

Connection to the MOE Sim
To download the Simulator, visit the [Releases Page](https://github.com/mahilab/MOE_sim/releases/tag/v0.2.0) of the [MOE Sim](https://github.com/mahilab/MOE_sim) repository. 

Most programs built using this MOE system include something like the following code, which enables the user to pass an argument `-v` when running an executable to connect to the simulator instead of the physical robot. For example, to run the `rom_demo` example, you would run `./rom_demo` to connect to the physical robot, or `./rom_demo -v` to connect to the simulator

```c++
/////////////////////////////////
// construct and config MOE   //
/////////////////////////////////

std::shared_ptr<Moe> moe = nullptr;
std::shared_ptr<Q8Usb> daq = nullptr;

if(result.count("virtual") > 0){
    MoeConfigurationVirtual config_vr; 
    moe = std::make_shared<MahiOpenExoVirtual>(config_vr);
}
else{
    daq = std::make_shared<Q8Usb>();
    daq->open();

    MoeConfigurationHardware config_hw(*daq,VelocityEstimator::Hardware); 

    std::vector<TTL> idle_values(8,TTL_LOW);
    daq->DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
    daq->DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
    daq->DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);   

    moe = std::make_shared<MahiOpenExoHardware>(config_hw);
}
```
