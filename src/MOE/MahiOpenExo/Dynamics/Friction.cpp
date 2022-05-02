#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
#include <Eigen/Dense>
using mahi::util::cos;
using mahi::util::sin;


namespace moe {
Eigen::VectorXd MoeDynamicModel::get_Friction(){
	Eigen::VectorXd friction = Eigen::VectorXd::Zero(4); 

	static std::vector<double> B_coeff = {0.0393, 0.0691, 0.0068, 0.0025};
	static std::vector<double> Fk_coeff = {0.1838, 0.1572, 0.0996, 0.1685};

	friction(0,0) = Fk_coeff[0]*std::tanh(qd[0]*10) + B_coeff[0]*qd[0];
	friction(1,0) = Fk_coeff[1]*std::tanh(qd[1]*10) + B_coeff[1]*qd[1];
	friction(2,0) = Fk_coeff[2]*std::tanh(qd[2]*10) + B_coeff[2]*qd[2];
	friction(3,0) = Fk_coeff[3]*std::tanh(qd[3]*10) + B_coeff[3]*qd[3];

	return friction;
}

} // namespace moe