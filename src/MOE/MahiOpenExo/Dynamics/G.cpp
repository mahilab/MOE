#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
#include <Eigen/Dense>
using mahi::util::cos;
using mahi::util::sin;


namespace moe {
Eigen::VectorXd MoeDynamicModel::get_G(){
	Eigen::VectorXd G = Eigen::VectorXd::Zero(4); 

	double q_s = 0;
	double dist = .28;
	const double t2 = cos(q[0]);
	const double t3 = cos(q[1]);
	const double t4 = cos(q[2]);
	const double t5 = cos(q[3]);
	const double t6 = cos(q_s);
	const double t7 = sin(q[0]);
	const double t8 = sin(q[1]);
	const double t9 = sin(q[2]);
	const double t10 = sin(q[3]);
	const double t11 = sin(q_s);
	G(0,0) = moe_mass_props.g*t6*(-moe_mass_props.J0.Pcx*moe_mass_props.J0.m*t7-moe_mass_props.J0.Pcy*moe_mass_props.J0.m*t2-moe_mass_props.J1.Pcz*moe_mass_props.J1.m*t2+dist*moe_mass_props.J1.m*t2+dist*moe_mass_props.J2.m*t2+dist*moe_mass_props.J3.m*t2-moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t2*t4+moe_mass_props.J1.Pcx*moe_mass_props.J1.m*t7*t8+moe_mass_props.J1.Pcy*moe_mass_props.J1.m*t3*t7+moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t2*t9-moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t3*t7+moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t9-moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t2*t4*t5+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t3*t7*t10-moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t7*t8*t9+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t3*t5*t7+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t4*t10-moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t4*t7*t8-moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t4*t7*t8-moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t5*t7*t8*t9+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t7*t8*t9*t10);
	G(1,0) = moe_mass_props.g*(moe_mass_props.J1.Pcx*moe_mass_props.J1.m*t8*t11+moe_mass_props.J1.Pcy*moe_mass_props.J1.m*t3*t11-moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t3*t11-moe_mass_props.J1.Pcx*moe_mass_props.J1.m*t2*t3*t6+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t3*t10*t11-moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t8*t9*t11+moe_mass_props.J1.Pcy*moe_mass_props.J1.m*t2*t6*t8+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t3*t5*t11-moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t4*t8*t11-moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t2*t6*t8-moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t4*t8*t11+moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t2*t3*t6*t9+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t2*t6*t8*t10-moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t5*t8*t9*t11+moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t2*t3*t4*t6+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t5*t6*t8+moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t8*t9*t10*t11+moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t3*t4*t6+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t2*t3*t5*t6*t9-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t3*t6*t9*t10);
	G(2,0) = moe_mass_props.g*(moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t3*t4*t11+moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t6*t7*t9+moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t4*t6*t7-moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t3*t9*t11+moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t4*t6*t7-moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t3*t9*t11+moe_mass_props.J2.Pcx*moe_mass_props.J2.m*t2*t4*t6*t8+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t3*t4*t5*t11+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t5*t6*t7*t9-moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t2*t6*t8*t9-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t3*t4*t10*t11-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t6*t7*t9*t10-moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t6*t8*t9+moe_mass_props.J3.Pcx*moe_mass_props.J3.m*t2*t4*t5*t6*t8-moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t4*t6*t8*t10);
	G(3,0) = -moe_mass_props.g*moe_mass_props.J3.m*(-moe_mass_props.J3.Pcx*t5*t8*t11+moe_mass_props.J3.Pcy*t8*t10*t11+moe_mass_props.J3.Pcx*t2*t3*t5*t6-moe_mass_props.J3.Pcx*t4*t6*t7*t10+moe_mass_props.J3.Pcx*t3*t9*t10*t11-moe_mass_props.J3.Pcy*t2*t3*t6*t10-moe_mass_props.J3.Pcy*t4*t5*t6*t7+moe_mass_props.J3.Pcy*t3*t5*t9*t11+moe_mass_props.J3.Pcx*t2*t6*t8*t9*t10+moe_mass_props.J3.Pcy*t2*t5*t6*t8*t9);

	return G;
}

} // namespace moe