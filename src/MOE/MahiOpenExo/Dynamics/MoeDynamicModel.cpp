#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
namespace moe {
    MoeDynamicModel::MoeDynamicModel() {

    }

    MoeDynamicModel::~MoeDynamicModel() {

    }
    void MoeDynamicModel::update(std::vector<double> joint_positions, std::vector<double> joint_velocities) {
        q = joint_positions;
        qd = joint_velocities;
    }

    void MoeDynamicModel::update_J0() {
        // x and y position of the counterweight and slider
        // double x_cw = cw_mass_props.Pcx;
        double y_cw = moe_mass_props.J0_counterweight.Pcy + (user_params.cw_location - 4)*0.01270000;
        // double x_sl = sl_mass_props.Pcx;
        double y_sl = moe_mass_props.J0_slider.Pcy + (user_params.forearm_location-3)*0.005;
        // calculate the new center of mass for moe_mass_props.J0
        double newPcx = (moe_mass_props.J0_counterweight.m*moe_mass_props.J0_counterweight.Pcx + moe_mass_props.J0_slider.m*moe_mass_props.J0_slider.Pcx + moe_mass_props.J0_main.m*moe_mass_props.J0_main.Pcx)/(moe_mass_props.J0_counterweight.m + moe_mass_props.J0_slider.m + moe_mass_props.J0_main.m);
        double newPcy = (moe_mass_props.J0_counterweight.m*y_cw + moe_mass_props.J0_slider.m*y_sl + moe_mass_props.J0_main.m*moe_mass_props.J0_main.Pcy)/(moe_mass_props.J0_counterweight.m + moe_mass_props.J0_slider.m + moe_mass_props.J0_main.m);
        double newPcz = (moe_mass_props.J0_counterweight.m*moe_mass_props.J0_counterweight.Pcz + moe_mass_props.J0_slider.m*moe_mass_props.J0_slider.Pcz + moe_mass_props.J0_main.m*moe_mass_props.J0_main.Pcz)/(moe_mass_props.J0_counterweight.m + moe_mass_props.J0_slider.m + moe_mass_props.J0_main.m);
        // Parallel Axis Thm (about new center of mass)
        // Icxx
        double Icxx_cw = moe_mass_props.J0_counterweight.Icxx + moe_mass_props.J0_counterweight.m*((y_cw-newPcy)*(y_cw-newPcy) + (moe_mass_props.J0_counterweight.Pcz-newPcz)*(moe_mass_props.J0_counterweight.Pcz-newPcz));
        double Icxx_sl = moe_mass_props.J0_slider.Icxx + moe_mass_props.J0_slider.m*((y_sl-newPcy)*(y_sl-newPcy) + (moe_mass_props.J0_slider.Pcz-newPcz)*(moe_mass_props.J0_slider.Pcz-newPcz));
        double Icxx_main = moe_mass_props.J0_main.Icxx + moe_mass_props.J0_main.m*((moe_mass_props.J0_main.Pcy-newPcy)*(moe_mass_props.J0_main.Pcy-newPcy) + (moe_mass_props.J0_main.Pcz-newPcz)*(moe_mass_props.J0_main.Pcz-newPcz));
        double newIcxx = Icxx_cw + Icxx_sl + Icxx_main;
        // Icyy
        double Icyy_cw = moe_mass_props.J0_counterweight.Icyy + moe_mass_props.J0_counterweight.m*((moe_mass_props.J0_counterweight.Pcx-newPcx)*(moe_mass_props.J0_counterweight.Pcx-newPcx) + (moe_mass_props.J0_counterweight.Pcz-newPcz)*(moe_mass_props.J0_counterweight.Pcz-newPcz));
        double Icyy_sl = moe_mass_props.J0_slider.Icyy + moe_mass_props.J0_slider.m*((moe_mass_props.J0_slider.Pcx-newPcx)*(moe_mass_props.J0_slider.Pcx-newPcx) + (moe_mass_props.J0_slider.Pcz-newPcz)*(moe_mass_props.J0_slider.Pcz-newPcz));
        double Icyy_main = moe_mass_props.J0_main.Icyy + moe_mass_props.J0_main.m*((moe_mass_props.J0_main.Pcx-newPcx)*(moe_mass_props.J0_main.Pcx-newPcx) + (moe_mass_props.J0_main.Pcz-newPcz)*(moe_mass_props.J0_main.Pcz-newPcz));
        double newIcyy = Icyy_cw + Icyy_sl + Icyy_main;
        
        // Iczz
        double Iczz_cw = moe_mass_props.J0_counterweight.Iczz + moe_mass_props.J0_counterweight.m*((moe_mass_props.J0_counterweight.Pcx-newPcx)*(moe_mass_props.J0_counterweight.Pcx-newPcx)+(y_cw-newPcy)*(y_cw-newPcy));
        double Iczz_sl = moe_mass_props.J0_slider.Iczz + moe_mass_props.J0_slider.m*((moe_mass_props.J0_slider.Pcx-newPcx)*(moe_mass_props.J0_slider.Pcx-newPcx)+(y_sl-newPcy)*(y_sl-newPcy));
        double Iczz_main = moe_mass_props.J0_main.Iczz + moe_mass_props.J0_main.m*((moe_mass_props.J0_main.Pcx-newPcx)*(moe_mass_props.J0_main.Pcx-newPcx)+(moe_mass_props.J0_main.Pcy-newPcy)*(moe_mass_props.J0_main.Pcy-newPcy));
        double newIczz = Iczz_cw + Iczz_sl + Iczz_main;

        // Icxy
        double Icxy_cw = moe_mass_props.J0_counterweight.Icxy + moe_mass_props.J0_counterweight.m*((moe_mass_props.J0_counterweight.Pcx-newPcx)*(y_cw-newPcy));
        double Icxy_sl = moe_mass_props.J0_slider.Icxy + moe_mass_props.J0_slider.m*((moe_mass_props.J0_slider.Pcx-newPcx)*(y_sl-newPcy));
        double Icxy_main = moe_mass_props.J0_main.Icxy + moe_mass_props.J0_main.m*((moe_mass_props.J0_main.Pcx-newPcx)*(moe_mass_props.J0_main.Pcy-newPcy));
        double newIcxy = Icxy_cw + Icxy_sl + Icxy_main;

        // Icxz
        double Icxz_cw = moe_mass_props.J0_counterweight.Icxz + moe_mass_props.J0_counterweight.m*((moe_mass_props.J0_counterweight.Pcx-newPcx)*(moe_mass_props.J0_counterweight.Pcz-newPcz));
        double Icxz_sl = moe_mass_props.J0_slider.Icxz + moe_mass_props.J0_slider.m*((moe_mass_props.J0_slider.Pcx-newPcx)*(moe_mass_props.J0_slider.Pcz-newPcz));
        double Icxz_main = moe_mass_props.J0_main.Icxz + moe_mass_props.J0_main.m*((moe_mass_props.J0_main.Pcx-newPcx)*(moe_mass_props.J0_main.Pcz-newPcz));
        double newIcxz = Icxz_cw + Icxz_sl + Icxz_main;

        // Icyz
        double Icyz_cw = moe_mass_props.J0_counterweight.Icyz + moe_mass_props.J0_counterweight.m*((moe_mass_props.J0_counterweight.Pcy-newPcy)*(moe_mass_props.J0_counterweight.Pcz-newPcz));
        double Icyz_sl = moe_mass_props.J0_slider.Icyz + moe_mass_props.J0_slider.m*((moe_mass_props.J0_slider.Pcy-newPcy)*(moe_mass_props.J0_slider.Pcz-newPcz));
        double Icyz_main = moe_mass_props.J0_main.Icyz + moe_mass_props.J0_main.m*((moe_mass_props.J0_main.Pcy-newPcy)*(moe_mass_props.J0_main.Pcz-newPcz));
        double newIcyz = Icyz_cw + Icyz_sl + Icyz_main;
        // Set new values to moe_mass_props.J0 (both the joint, and all_props)
        moe_mass_props.J0.Pcx = newPcx;
        moe_mass_props.J0.Pcy = newPcy;
        moe_mass_props.J0.Pcz = newPcz;
        moe_mass_props.J0.Icxx = newIcxx;
        moe_mass_props.J0.Icyy = newIcyy;
        moe_mass_props.J0.Iczz = newIczz;
        moe_mass_props.J0.Icxy = newIcxy;
        moe_mass_props.J0.Icxz = newIcxz;
        moe_mass_props.J0.Icyz = newIcyz;

        
    }

    void MoeDynamicModel::set_user_params(UserParams newParams) {
        user_params = newParams;
        update_J0();
    }

    UserParams MoeDynamicModel::get_user_params() {
        return user_params;
    }

}