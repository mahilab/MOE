#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
#include <casadi/casadi.hpp>
using mahi::util::cos;
using mahi::util::sin;


namespace moe {
casadi::MX MoeDynamicModel::cas_get_Friction_m(){
	casadi::MX friction = casadi::MX::zeros(4,1); 

	friction(0,0) = Fk_coeff[0]*std::tanh(qdm[0]*10.0) + B_coeff[0]*qdm[0];
	friction(1,0) = Fk_coeff[1]*std::tanh(qdm[1]*10.0) + B_coeff[1]*qdm[1];
	friction(2,0) = Fk_coeff[2]*std::tanh(qdm[2]*10.0) + B_coeff[2]*qdm[2];
	friction(3,0) = Fk_coeff[3]*std::tanh(qdm[3]*10.0) + B_coeff[3]*qdm[3];

	return friction;
}

} // namespace moe