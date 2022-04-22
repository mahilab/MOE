#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>

using mahi::util::cos;
using mahi::util::sin;

namespace moe {
inline Eigen::VectorXd MoeDynamicModel::get_G(){
	Eigen::VectorXd G = Eigen::VectorXd::Zero(4); 

	const double t2 = cos(q[0]);
	const double t3 = cos(q[1]);
	const double t4 = cos(q[2]);
	const double t5 = cos(q[3]);
	const double t6 = sin(q[0]);
	const double t7 = sin(q[1]);
	const double t8 = sin(q[2]);
	const double t9 = sin(q[3]);
	G(0,0) = g*(moe_mass_props.J1.m*t2*2.8024836E-1+moe_mass_props.J2.m*t2*2.8024836E-1+moe_mass_props.J3.m*t2*2.8024836E-1-moe_mass_props.J0.Pcx*moe_mass_props.J0.m*t6-moe_mass_props.J0.Pcy*moe_mass_props.J0.m*t2-moe_mass_props.J1.Pcz*moe_mass_props.J1.m*t2-moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t2*t4+moe_mass_props.J1.Pcx*moe_mass_props.J1.m*t6*t7+moe_mass_props.J1.Pcy*moe_mass_props.J1.m*t3*t6+moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t2*t8-moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t3*t6+moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t8-moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t2*t4*t5+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t3*t6*t9-moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t6*t7*t8+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t3*t5*t6-moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t4*t6*t7+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t4*t9-moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t4*t6*t7-moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t5*t6*t7*t8+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t6*t7*t8*t9);
	G(1,0) = g*t2*(-moe_mass_props.J1.Pcx*moe_mass_props.J1.m*t3+moe_mass_props.J1.Pcy*moe_mass_props.J1.m*t7-moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t7+moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t3*t8+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t7*t9+moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t3*t4+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t5*t7+moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t3*t4+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t3*t5*t8-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t3*t8*t9);
	G(2,0) = g*(moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t6*t8+moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t4*t6+moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t4*t6+moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t2*t4*t7+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t5*t6*t8-moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t2*t7*t8-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t6*t8*t9-moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t7*t8+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t2*t4*t5*t7-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t4*t7*t9);
	G(3,0) = -g*moe_mass_props.J3.m*(moe_mass_props.J3.Pcx*t2*t3*t5-moe_mass_props.J3.Pcx*t4*t6*t9-moe_mass_props.J3.Pcy*t2*t3*t9-moe_mass_props.J3.Pcy*t4*t5*t6+moe_mass_props.J3.Pcx*t2*t7*t8*t9+moe_mass_props.J3.Pcy*t2*t5*t7*t8);

	return G;
}

} // namespace moe