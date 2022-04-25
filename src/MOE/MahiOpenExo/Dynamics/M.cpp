#include <Moe/MahiOpenExo/Dynamics/MoeDynamicModel.hpp>
#include <Mahi/Util.hpp>
using mahi::util::cos;
using mahi::util::sin;

namespace moe {
inline Eigen::MatrixXd MoeDynamicModel::get_M(){
	Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4,4); 

	const double t2 = cos(q[1]);
	const double t3 = cos(q[2]);
	const double t4 = cos(q[3]);
	const double t5 = sin(q[1]);
	const double t6 = sin(q[2]);
	const double t7 = sin(q[3]);
	const double t8 = moe_mass_props.J1.Pcx*moe_mass_props.J1.Pcx;
	const double t9 = moe_mass_props.J2.Pcx*moe_mass_props.J2.Pcx;
	const double t10 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcx;
	const double t11 = moe_mass_props.J1.Pcy*moe_mass_props.J1.Pcy;
	const double t12 = moe_mass_props.J2.Pcy*moe_mass_props.J2.Pcy;
	const double t13 = moe_mass_props.J3.Pcy*moe_mass_props.J3.Pcy;
	const double t14 = moe_mass_props.J2.Pcz*moe_mass_props.J2.Pcz;
	const double t15 = moe_mass_props.J3.Pcz*moe_mass_props.J3.Pcz;
	const double t16 = dist*dist;
	const double t17 = q[1]*2.0;
	const double t18 = q[2]*2.0;
	const double t19 = q[3]*2.0;
	const double t20 = t2*t2;
	const double t21 = t3*t3;
	const double t22 = t4*t4;
	const double t23 = sin(t17);
	const double t24 = sin(t18);
	const double t25 = sin(t19);
	const double t26 = moe_mass_props.J2.Icxy*t2;
	const double t27 = moe_mass_props.J1.Icxz*t2;
	const double t28 = moe_mass_props.J3.Icxy*t3;
	const double t29 = moe_mass_props.J2.Icxz*t3;
	const double t30 = moe_mass_props.J3.Icyz*t4;
	const double t31 = moe_mass_props.J3.Icxx*t5;
	const double t32 = moe_mass_props.J3.Icxz*t7;
	const double t33 = moe_mass_props.J1.Icyz*t5;
	const double t34 = moe_mass_props.J2.Icyz*t6;
	const double t35 = moe_mass_props.J2.Iczz*t5;
	const double t36 = moe_mass_props.J3.Iczz*t6;
	const double t37 = moe_mass_props.J1.m*t8;
	const double t38 = moe_mass_props.J2.m*t9;
	const double t39 = moe_mass_props.J3.m*t10;
	const double t40 = moe_mass_props.J2.m*t12;
	const double t41 = moe_mass_props.J3.m*t13;
	const double t42 = moe_mass_props.J3.m*t15;
	const double t43 = moe_mass_props.J2.Pcx*moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t2;
	const double t44 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t3;
	const double t45 = moe_mass_props.J1.Pcx*moe_mass_props.J1.Pcz*moe_mass_props.J1.m*t2;
	const double t46 = moe_mass_props.J2.Pcx*moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t3;
	const double t47 = moe_mass_props.J3.Pcy*moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t4;
	const double t48 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t7;
	const double t49 = moe_mass_props.J1.Pcy*moe_mass_props.J1.Pcz*moe_mass_props.J1.m*t5;
	const double t50 = moe_mass_props.J2.Pcy*moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t6;
	const double t51 = moe_mass_props.J3.Icxz*t2*t4;
	const double t52 = moe_mass_props.J3.Icxz*t3*t4;
	const double t53 = moe_mass_props.J2.Icyz*t2*t3;
	const double t54 = moe_mass_props.J3.Iczz*t2*t3;
	const double t55 = moe_mass_props.J1.Pcx*dist*moe_mass_props.J1.m*t2;
	const double t56 = moe_mass_props.J3.Icxy*t2*t6;
	const double t58 = moe_mass_props.J2.Icxz*t2*t6;
	const double t60 = moe_mass_props.J3.Icyz*t2*t7;
	const double t62 = moe_mass_props.J3.Icyz*t3*t7;
	const double t64 = moe_mass_props.J1.Pcy*dist*moe_mass_props.J1.m*t5;
	const double t65 = moe_mass_props.J2.Pcz*dist*moe_mass_props.J2.m*t5;
	const double t84 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t4;
	const double t85 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t3*t4;
	const double t86 = moe_mass_props.J2.Pcy*moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t2*t3;
	const double t87 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t2*t6;
	const double t89 = moe_mass_props.J2.Pcx*moe_mass_props.J2.Pcz*moe_mass_props.J2.m*t2*t6;
	const double t91 = moe_mass_props.J3.Pcy*moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t2*t7;
	const double t93 = moe_mass_props.J3.Pcy*moe_mass_props.J3.Pcz*moe_mass_props.J3.m*t3*t7;
	const double t96 = moe_mass_props.J3.Pcx*dist*moe_mass_props.J3.m*t2*t4;
	const double t97 = moe_mass_props.J2.Pcy*dist*moe_mass_props.J2.m*t2*t3;
	const double t98 = moe_mass_props.J3.Pcz*dist*moe_mass_props.J3.m*t2*t3;
	const double t102 = moe_mass_props.J2.Icxx*t2*t3*t6;
	const double t103 = moe_mass_props.J3.Icxx*t3*t4*t7;
	const double t106 = moe_mass_props.J2.Icyy*t2*t3*t6;
	const double t107 = moe_mass_props.J3.Icyy*t2*t3*t6;
	const double t108 = moe_mass_props.J3.Icyy*t3*t4*t7;
	const double t110 = moe_mass_props.J2.Pcx*dist*moe_mass_props.J2.m*t2*t6;
	const double t111 = moe_mass_props.J2.Pcx*dist*moe_mass_props.J2.m*t3*t5;
	const double t112 = moe_mass_props.J3.Pcy*dist*moe_mass_props.J3.m*t2*t7;
	const double t113 = moe_mass_props.J3.Pcy*dist*moe_mass_props.J3.m*t4*t5;
	const double t118 = moe_mass_props.J3.Pcx*dist*moe_mass_props.J3.m*t5*t7;
	const double t119 = moe_mass_props.J2.Pcy*dist*moe_mass_props.J2.m*t5*t6;
	const double t120 = moe_mass_props.J3.Pcz*dist*moe_mass_props.J3.m*t5*t6;
	const double t142 = moe_mass_props.J3.Icxy*t4*t5*t7*2.0;
	const double t156 = moe_mass_props.J3.Pcx*dist*moe_mass_props.J3.m*t3*t4*t5;
	const double t160 = moe_mass_props.J3.Icxx*t2*t4*t6*t7;
	const double t162 = moe_mass_props.J3.Icyy*t2*t4*t6*t7;
	const double t165 = moe_mass_props.J3.Pcy*dist*moe_mass_props.J3.m*t3*t5*t7;
	const double t195 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t4*t5*t7*2.0;
	const double t57 = t5*t28;
	const double t59 = t5*t29;
	const double t61 = t5*t30;
	const double t63 = t6*t30;
	const double t66 = t5*t32;
	const double t67 = t6*t32;
	const double t68 = t5*t34;
	const double t69 = -t28;
	const double t70 = -t29;
	const double t71 = -t31;
	const double t72 = -t33;
	const double t73 = -t35;
	const double t74 = -t36;
	const double t75 = moe_mass_props.J3.Icxx*t22;
	const double t76 = moe_mass_props.J3.Icyy*t22;
	const double t77 = moe_mass_props.J3.Icxy*t25;
	const double t78 = t5*t38;
	const double t79 = t6*t39;
	const double t80 = t5*t40;
	const double t81 = t5*t41;
	const double t82 = t6*t41;
	const double t83 = t5*t42;
	const double t88 = t5*t44;
	const double t90 = t5*t46;
	const double t92 = t5*t47;
	const double t94 = t6*t47;
	const double t95 = t2*t3*t30;
	const double t99 = t5*t48;
	const double t100 = t6*t48;
	const double t101 = t5*t50;
	const double t104 = t2*t3*t32;
	const double t105 = t6*t51;
	const double t109 = t2*t3*t36;
	const double t114 = -t44;
	const double t115 = -t46;
	const double t116 = t6*t60;
	const double t121 = -t49;
	const double t122 = -t52;
	const double t123 = -t53;
	const double t124 = -t55;
	const double t126 = -t56;
	const double t127 = -t58;
	const double t128 = -t60;
	const double t131 = -t65;
	const double t135 = moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t25;
	const double t136 = t22*t31;
	const double t140 = t2*t3*t39;
	const double t141 = t2*t3*t41;
	const double t149 = t22*t39;
	const double t150 = t22*t41;
	const double t151 = t21*t26*2.0;
	const double t152 = t22*t28*2.0;
	const double t153 = t6*t91;
	const double t155 = t6*t96;
	const double t157 = -t85;
	const double t158 = -t86;
	const double t161 = t3*t4*t7*t31;
	const double t163 = t5*t108;
	const double t164 = t6*t112;
	const double t166 = t6*t113;
	const double t167 = -t87;
	const double t168 = -t89;
	const double t169 = -t91;
	const double t172 = -t96;
	const double t173 = t6*t118;
	const double t177 = -t103;
	const double t179 = -t106;
	const double t181 = -t119;
	const double t182 = -t120;
	const double t186 = t2*t3*t47;
	const double t187 = t2*t3*t48;
	const double t188 = t6*t84;
	const double t189 = t2*t3*t6*t38;
	const double t190 = t3*t4*t7*t39;
	const double t191 = t2*t3*t6*t40;
	const double t193 = t3*t4*t7*t41;
	const double t194 = t2*t3*t6*t42;
	const double t196 = t21*t43*2.0;
	const double t197 = t22*t44*2.0;
	const double t198 = t21*t51*2.0;
	const double t201 = t22*t56*2.0;
	const double t203 = t21*t60*2.0;
	const double t205 = -t160;
	const double t208 = -t165;
	const double t220 = t21*t84*2.0;
	const double t221 = t22*t87*2.0;
	const double t223 = t21*t91*2.0;
	const double t229 = t2*t4*t6*t7*t28*2.0;
	const double t235 = t2*t4*t6*t7*t44*2.0;
	const double t239 = t30+t32+t47+t48;
	const double t117 = t6*t61;
	const double t125 = t6*t66;
	const double t129 = -t61;
	const double t130 = -t63;
	const double t132 = -t66;
	const double t133 = -t67;
	const double t134 = -t68;
	const double t137 = t5*t76;
	const double t138 = -t75;
	const double t139 = -t77;
	const double t143 = -t78;
	const double t144 = -t79;
	const double t145 = -t80;
	const double t146 = -t81;
	const double t147 = -t82;
	const double t148 = -t83;
	const double t154 = t6*t92;
	const double t159 = t6*t99;
	const double t170 = -t92;
	const double t171 = -t94;
	const double t174 = -t99;
	const double t175 = -t100;
	const double t176 = -t101;
	const double t178 = -t105;
	const double t180 = t2*t3*t74;
	const double t183 = -t135;
	const double t184 = -t151;
	const double t192 = t2*t3*t82;
	const double t199 = t5*t149;
	const double t200 = t22*t81;
	const double t202 = t22*t57*2.0;
	const double t204 = -t150;
	const double t206 = -t163;
	const double t207 = -t164;
	const double t209 = -t166;
	const double t210 = -t173;
	const double t211 = t2*t3*t6*t75;
	const double t212 = t2*t3*t6*t76;
	const double t213 = -t196;
	const double t214 = -t198;
	const double t216 = -t188;
	const double t217 = -t189;
	const double t219 = -t193;
	const double t222 = t22*t88*2.0;
	const double t225 = t2*t4*t7*t79;
	const double t226 = t5*t190;
	const double t227 = t2*t4*t7*t82;
	const double t228 = t3*t4*t7*t81;
	const double t230 = -t220;
	const double t233 = t2*t3*t22*t79;
	const double t185 = -t137;
	const double t215 = -t202;
	const double t218 = t2*t3*t147;
	const double t224 = -t199;
	const double t231 = -t222;
	const double t232 = -t212;
	const double t234 = t22*t192;
	const double t236 = -t226;
	const double t237 = t2*t4*t7*t147;
	const double t238 = t2*t3*t22*t144;
	const double t240 = t62+t74+t93+t122+t144+t147+t157;
	const double t241 = t34+t50+t69+t70+t108+t114+t115+t130+t133+t152+t171+t175+t177+t190+t197+t219;
	const double t242 = t54+t112+t116+t129+t132+t140+t141+t153+t170+t172+t174+t178+t209+t210+t216;
	const double t243 = t71+t73+t95+t104+t111+t123+t126+t127+t136+t142+t143+t145+t146+t148+t156+t158+t162+t167+t168+t181+t182+t185+t186+t187+t195+t200+t201+t205+t208+t221+t224+t225+t237;
	const double t244 = t26+t27+t43+t45+t51+t57+t59+t64+t72+t84+t88+t90+t97+t98+t102+t107+t110+t113+t117+t118+t121+t124+t125+t128+t131+t134+t154+t155+t159+t161+t169+t176+t179+t180+t184+t191+t194+t203+t206+t207+t211+t213+t214+t215+t217+t218+t223+t228+t229+t230+t231+t232+t234+t235+t236+t238;
	M(0,0) = moe_mass_props.J3.Icxx+moe_mass_props.J1.Icyy+moe_mass_props.J0.Iczz+moe_mass_props.J2.Iczz+t37+t38+t40+t41+t42+t76+t138+t139+t149+t183+t204+moe_mass_props.J1.Icxx*t20+moe_mass_props.J2.Icxx*t20-moe_mass_props.J3.Icxx*t20+moe_mass_props.J1.Icxy*t23-moe_mass_props.J1.Icyy*t20+moe_mass_props.J3.Icyy*t20-moe_mass_props.J2.Iczz*t20+moe_mass_props.J1.m*t16+moe_mass_props.J2.m*t16+moe_mass_props.J3.m*t16-t20*t37+t5*t53*2.0-t20*t38+t20*t39+t5*t56*2.0-t20*t41+t5*t58*2.0+t5*t86*2.0+t5*t87*2.0+t5*t89*2.0+t20*t75*2.0-t20*t76*2.0-t5*t162*2.0-t20*t149*2.0+t20*t150*2.0-t5*t225*2.0+(moe_mass_props.J0.Pcx*moe_mass_props.J0.Pcx)*moe_mass_props.J0.m+(moe_mass_props.J0.Pcy*moe_mass_props.J0.Pcy)*moe_mass_props.J0.m+(moe_mass_props.J1.Pcz*moe_mass_props.J1.Pcz)*moe_mass_props.J1.m-moe_mass_props.J1.Pcz*dist*moe_mass_props.J1.m*2.0-moe_mass_props.J2.Icxx*t20*t21+moe_mass_props.J2.Icyy*t20*t21-moe_mass_props.J3.Icyy*t20*t21+moe_mass_props.J3.Iczz*t20*t21+moe_mass_props.J1.m*t11*t20+moe_mass_props.J2.m*t14*t20-t2*t3*t61*2.0-t2*t3*t66*2.0-t6*t20*t52*2.0+t20*t21*t38-t20*t21*t40+t20*t21*t41-t5*t22*t56*4.0-t20*t21*t42+t6*t20*t62*2.0-t2*t3*t92*2.0-t2*t3*t99*2.0-t6*t20*t85*2.0-t5*t22*t87*4.0+t20*t21*t76+t6*t20*t93*2.0+t20*t21*t138+t20*t21*t149+t20*t21*t204-moe_mass_props.J2.Icxy*t3*t6*t20*2.0+moe_mass_props.J3.Icxy*t4*t7*t20*4.0+moe_mass_props.J1.Pcx*moe_mass_props.J1.Pcy*moe_mass_props.J1.m*t23-moe_mass_props.J2.Pcx*dist*moe_mass_props.J2.m*t3*2.0+moe_mass_props.J2.Pcy*dist*moe_mass_props.J2.m*t6*2.0+moe_mass_props.J3.Pcz*dist*moe_mass_props.J3.m*t6*2.0-moe_mass_props.J3.Pcx*dist*moe_mass_props.J3.m*t3*t4*2.0+moe_mass_props.J3.Pcy*dist*moe_mass_props.J3.m*t3*t7*2.0-moe_mass_props.J3.Icxy*t4*t7*t20*t21*2.0+t2*t4*t6*t7*t31*2.0+t2*t4*t6*t7*t81*2.0-moe_mass_props.J2.Pcx*moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t3*t6*t20*2.0+moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t4*t7*t20*4.0-moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t4*t7*t20*t21*2.0;
	M(0,1) = t244;
	M(0,2) = t243;
	M(0,3) = t242;
	M(1,0) = t244;
	M(1,1) = moe_mass_props.J2.Icyy+moe_mass_props.J1.Iczz+moe_mass_props.J3.Iczz+t37+t38+t39+t41+moe_mass_props.J2.Icxx*t21+moe_mass_props.J2.Icxy*t24-moe_mass_props.J2.Icyy*t21+moe_mass_props.J3.Icyy*t21-moe_mass_props.J3.Iczz*t21+moe_mass_props.J1.m*t11+moe_mass_props.J2.m*t14+t6*t52*2.0-t21*t38+t21*t40-t21*t41+t21*t42-t6*t62*2.0+t6*t85*2.0+t21*t75-t21*t76-t6*t93*2.0-t21*t149+t21*t150+moe_mass_props.J3.Icxy*t4*t7*t21*2.0+moe_mass_props.J2.Pcx*moe_mass_props.J2.Pcy*moe_mass_props.J2.m*t24+moe_mass_props.J3.Pcx*moe_mass_props.J3.Pcy*moe_mass_props.J3.m*t4*t7*t21*2.0;
	M(1,2) = t241;
	M(1,3) = t240;
	M(2,0) = t243;
	M(2,1) = t241;
	M(2,2) = moe_mass_props.J3.Icxx+moe_mass_props.J2.Iczz+t38+t40+t41+t42+t76+t138+t139+t149+t183+t204;
	M(2,3) = t239;
	M(3,0) = t242;
	M(3,1) = t240;
	M(3,2) = t239;
	M(3,3) = moe_mass_props.J3.Iczz+t39+t41;

	return M;
}

} // namespace moe