#ifndef ARM_DEFINE_HPP
#define ARM_DEFINE_HPP

const int ARM_JOINT_NUM = 3;

const double LINK_LENGTH[ARM_JOINT_NUM] 
    = {0.3, 0.3, 0.06}; // link1, link2, link3 [m]

const std::string chaser_urdf_path_ = 
"/home/siel/arm_ws/model/Chaser_URDF/miniPFR.urdf";
const std::string chaser_ee_name_ = "ee_site";



#endif // ARM_DEFINE_HPP