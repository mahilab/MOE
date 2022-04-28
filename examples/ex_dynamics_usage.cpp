#include <MOE/MOE.hpp>

using namespace moe;

int main(int argc, char* argv[]) {
    MoeDynamicModel moe_dyn_model;

    moe_dyn_model.set_user_params(UserParams{3,4,0});
    moe_dyn_model.update({0,0,0,0},{0,0,0,0});
    moe_dyn_model.update_J0();
    std::cout << moe_dyn_model.q << std::endl;
    std::cout << moe_dyn_model.qd << std::endl;
    std::cout << moe_dyn_model.get_G();
    // std::cout << test_G;
    
    // std::cout << moe_dyn_model.get_Friction() << std::endl;

    // moe_dyn_model.update({0.234,-6.1,4.23,0.3},{3.6,1.2,-2.6,-0.2});
    
    // std::cout << moe_dyn_model.get_M() << std::endl;
    // std::cout << moe_dyn_model.get_V() << std::endl;
    // std::cout << moe_dyn_model.get_G() << std::endl;
    // std::cout << moe_dyn_model.get_Friction() << std::endl;

    return 0;
}
