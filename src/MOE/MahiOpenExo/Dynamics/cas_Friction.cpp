#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
#include <casadi/casadi.hpp>
using mahi::util::cos;
using mahi::util::sin;


namespace moe {
casadi::SX MoeDynamicModel::cas_get_Friction(){
	casadi::SX friction = casadi::SX::zeros(4,1); 

	friction(0,0) = Fk_coeff[0]*tanh(cas_qd(0)*10.0) + B_coeff[0]*cas_qd(0);
	friction(1,0) = Fk_coeff[1]*tanh(cas_qd(1)*10.0) + B_coeff[1]*cas_qd(1);
	friction(2,0) = Fk_coeff[2]*tanh(cas_qd(2)*10.0) + B_coeff[2]*cas_qd(2);
	friction(3,0) = Fk_coeff[3]*tanh(cas_qd(3)*10.0) + B_coeff[3]*cas_qd(3);

	return friction;
}

} // namespace moe