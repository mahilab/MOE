#pragma once
#include <array>
#include <Mahi/Util.hpp>
using namespace mahi::util;
namespace moe{
    struct JointProperties
    {
        double m;
        double Pcx;
        double Pcy;
        double Pcz;
        double Icxx;
        double Icyy;
        double Iczz;
        double Icxy;
        double Icxz;
        double Icyz;
        static JointProperties get_from_json(std::string filename) {
            json arm;
            std::ifstream armFile(filename);
            armFile >> arm;

            JointProperties arm_joint_properties;
            arm_joint_properties.m = arm["m"].get<double>();
            arm_joint_properties.Pcx = arm["Pcx"].get<double>();
            arm_joint_properties.Pcy = arm["Pcy"].get<double>();
            arm_joint_properties.Pcz = arm["Pcz"].get<double>();
            arm_joint_properties.Icxx = arm["Icxx"].get<double>();
            arm_joint_properties.Icyy = arm["Icyy"].get<double>();
            arm_joint_properties.Iczz = arm["Iczz"].get<double>();
            arm_joint_properties.Icxy = arm["Icxy"].get<double>();
            arm_joint_properties.Icxz = arm["Icxz"].get<double>();
            arm_joint_properties.Icyz = arm["Icyz"].get<double>();

            return arm_joint_properties;
        }
    };
    
 
    

}