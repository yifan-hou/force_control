#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>
#include <string>

#include <unsupported/Eigen/MatrixFunctions>

#include <forcecontrol/utilities.h>


typedef std::chrono::high_resolution_clock Clock;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

void truncate3f(Vector3f *v, float max, float min)
{
    for(int i=0; i<3; i++)
    {
        (*v)[i] = ((*v)[i] > max)? max:(*v)[i];
        (*v)[i] = ((*v)[i] < min)? min:(*v)[i];
    }
}

void stream_array_in3f(ostream &st, const Vector3f array)
{
    for (int i = 0; i<3; i++)
    {
        st << array(i);
        st << "\t";
    }
}

/*  Frames/spaces:
        W: world frame
        T: current tool frame
        So: set tool frame with offset
        Tf: transformed generalized space
    Quantities:
        SE3: 4x4 homogeneous coordinates
        se3: 6x1 twist coordinate of SE3
        spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for rotation
        v: 6x1 velocity, either spatial or body
        td: 6x1 time derivative of twist. It is also the generalized velocity

        wrench: 6x1 wrench. Makes work with body velocity
        GF: 6x1 generalized force. Makes work with generalized velocity
*/


Matrix3f wedge(const Vector3f &v) {
  Matrix3f v_wedge;
  v_wedge << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
  return v_wedge;
}

Matrix4f wedge6(const Vector6f &t) {
  Matrix4f t_wedge;
  t_wedge <<   0   -t(5)   t(4)  t(0),
                t(5)     0   -t(3)  t(1),
               -t(4)   t(3)     0   t(2),
                  0      0      0     0;
  return t_wedge;
}

Matrix3f quat2SO3(const Eigen::Quaternionf &q) {
  float q11 = q.x()*q.x();
  float q22 = q.y()*q.y();
  float q33 = q.z()*q.z();
  float q01 = q.w()*q.x();
  float q02 = q.w()*q.y();
  float q03 = q.w()*q.z();
  float q12 = q.x()*q.y();
  float q13 = q.x()*q.z();
  float q23 = q.y()*q.z();

  Matrix3f m;
  m << 1.0f - 2.0f*q22 - 2.0f*q33,  2.0f*(q12 - q03),      2.0f*(q13 + q02),
       2.0f*(q12 + q03),     1.0f - 2.0f*q11 - 2.0f*q33,  2.0f*(q23 - q01),
       2.0f*(q13 - q02),     2.0f*(q23 + q01),      1.0f - 2.0f*q11 - 2.0f*q22;
  return m;
}

Matrix3f quat2SO3(float qw, float qx, float qy, float qz) {
  float q11 = qx*qx;
  float q22 = qy*qy;
  float q33 = qz*qz;
  float q01 = qw*qx;
  float q02 = qw*qy;
  float q03 = qw*qz;
  float q12 = qx*qy;
  float q13 = qx*qz;
  float q23 = qy*qz;

  Matrix3f m;
  m << 1.0f - 2.0f*q22 - 2.0f*q33, 2.0f*(q12 - q03),      2.0f*(q13 + q02),
      2.0f*(q12 + q03),     1.0f - 2.0f*q11 - 2.0f*q33,  2.0f*(q23 - q01),
      2.0f*(q13 - q02),     2.0f*(q23 + q01),      1.0f - 2.0f*q11 - 2.0f*q22;
  return m;
}

Matrix3f so32SO3(const Vector3f &v) {
    return wedge(v).exp();
}

Vector3f SO32so3(const Matrix3f &R) {
    Vector3f so3;
    float theta = std::acos((R.trace()-1.0f)/2.0f);
    if(fabs(theta) < 1e-5) {
        so3(0) = 1.0f;
        so3(1) = 0.0f;
        so3(2) = 0.0f;
    } else {
        so3(0) = R(2,1)-R(1,2);
        so3(1) = R(0,2)-R(2,0);
        so3(2) = R(1,0)-R(0,1);
        so3 /= 2.0f*sin(theta);
    }
    so3 *= theta;
    return so3;
}

void so32quat(const Vector3f &so3, float *q) {
    float theta = so3.norm();
    if (theta < 1e-5) {
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
    } else {
        // q = [cos(theta/2); sin(theta/2)*so3/theta];
        float sin_theta = std::sin(theta/2.0f)/theta;
        q[0] = std::cos(theta/2.0f);
        q[1] = so3(0)*sin_theta;
        q[2] = so3(1)*sin_theta;
        q[3] = so3(2)*sin_theta;
    }
}
void SO32quat(const Matrix3f &SO3, float *q) {
    so32quat(SO32so3(SO3), q);
}

Matrix4f pose2SE3(const float *pose) {
    Matrix4f SE3 = Matrix4f::Identity();
    SE3(0, 3) = pose[0];
    SE3(1, 3) = pose[1];
    SE3(2, 3) = pose[2];
    SE3.block<3,3>(0,0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
    return SE3;
}

Matrix4f se32SE3(const Vector6f &twist) {
    Matrix4f SE3 = Matrix4f::Identity();
    float theta = twist.tail(3).norm();
    if ( theta < 1e-6 ) {
        // no rotation
        SE3(0, 3) = twist(0);
        SE3(1, 3) = twist(1);
        SE3(2, 3) = twist(2);
    } else {
        Vector3f v = twist.head(3);
        Vector3f w = twist.tail(3);
        Matrix3f R = so32SO3(w);
        v /= theta;
        w /= theta;
        SE3.block<3,3>(0, 0) = R;
        SE3.block<3,1>(0, 3) = (Matrix3f::Identity() - R)*(w.cross(v)) +
                w*w.transpose()*v*theta;
    }
    return SE3;
}

Matrix4f spt2SE3(const Vector6f &spt) {
    Matrix4f SE3 = Matrix4f::Identity();
    SE3.block<3, 3>(0, 0) = so32SO3(spt.tail(3));
    SE3.block<3, 1>(0, 3) = spt.head(3);
    return SE3;
}

Matrix4f SE3Inv(const Matrix4f &SE3) {
    Matrix4f SE3_inv = Matrix4f::Identity();
    SE3_inv.block<3,1>(0, 3) =
            -SE3.block<3,3>(0,0).transpose()*SE3.block<3,1>(0,3);
    SE3_inv.block<3,3>(0,0) = SE3.block<3,3>(0,0).transpose();
    return SE3_inv;
}

Vector6f SE32se3(const Matrix4f &SE3) {
    Vector3f p     = SE3.block<3,1>(0, 3);
    Vector3f omega = SO32so3(SE3.block<3,3>(0,0));
    float theta = omega.norm();
    if (theta < 1e-5) {
        Vector6f se3;
        se3 << p(0), p(1), p(2), 0, 0, 0;
        return se3;
    } else {
        omega /= theta;
        Matrix3f M =
                (Matrix3f::Identity() - wedge(omega*theta).exp())*
                wedge(omega)+omega*omega.transpose()*theta;
        Vector6f se3;
        se3.head(3) = M.fullPivLu().solve(p);
        se3.tail(3) = omega;
        se3 *= theta;
        return se3;
    }
}

Vector6f SE32spt(const Matrix4f &SE3) {
    Vector6f spt;
    spt.head(3) = SE3.block<3, 1>(0, 3);
    spt.tail(3) = SO32so3(SE3.block<3,3>(0,0));
    return spt;
}

Matrix6f SE32Adj(const Matrix4f &SE3) {
    Matrix6f Adj;
    Adj.topLeftCorner(3, 3) = SE3.topLeftCorner(3, 3);
    Adj.bottomRightCorner(3, 3) = SE3.topLeftCorner(3, 3);
    Adj.topRightCorner(3, 3) =
            wedge(SE3.block<3,1>(0, 3)) * SE3.topLeftCorner(3, 3);
    return Adj;
}

void SE32Pose(const Matrix4f &SE3, float *pose) {
    pose[0] = SE3(0, 3);
    pose[1] = SE3(1, 3);
    pose[2] = SE3(2, 3);
    SO32quat(SE3.block<3,3>(0,0), pose + 3);
}

ForceControlController::ForceControlController()
{
    _T = Matrix3f::Identity();
    _pose_user_input    = new float[7];
    _pose_offset        = new float[3]{0};
    _pose_sent_to_robot = new float[7];

    _wrench_Tr_set = Vector3f::Zero();
    _v_T        = Vector3f::Zero();
    _v_T_old    = Vector3f::Zero();
    _p_W        = Vector3f::Zero();
}

ForceControlController::~ForceControlController()
{
    delete [] _pose_user_input;
    delete [] _pose_offset;
    delete [] _pose_sent_to_robot;

    if (_print_flag)
        _file.close();
}

bool ForceControlController::init(ros::NodeHandle& root_nh, ForceControlHardware* hw, std::chrono::high_resolution_clock::time_point time0)
{
    _hw    = hw;
    _time0 = time0;

    // read set pose
    float wrench[6];
    _hw->getState(_pose_user_input, wrench);
    std::cout << "[ForceControlController] set pose: " << std::endl;
    UT::stream_array_in(cout, _pose_user_input, 7);
    std::cout << std::endl;
    std::cout << "[ForceControlController] set force: " << std::endl;
    std::cout << _wrench_Tr_set(0) << ", " << _wrench_Tr_set(1) << ", " << _wrench_Tr_set(2);
    std::cout << std::endl;

    // read controller from parameter server
    float AC_para_mass[3], AC_para_alpha[3];
    float fHz;
    std::vector<float> T_elements;
    string fullpath;
    root_nh.getParam("/Transform", T_elements);
    root_nh.param(string("/main_loop_rate"), fHz, 500.0f);
    root_nh.param(string("/force_controlled_dimensions"), _n_af, 3);
    root_nh.param(string("/FC_gains/PGain"), _kForceControlPGain, 1.0f);
    root_nh.param(string("/FC_gains/IGain"), _kForceControlIGain, 0.0f);
    root_nh.param(string("/FC_gains/DGain"), _kForceControlDGain, 0.0f);
    root_nh.param(string("/FC_I_Limit"), _FC_I_Limit, 10.0f);

    root_nh.param(string("/forcecontrol_print_flag"), _print_flag, false);
    root_nh.param(string("/forcecontrol_file_path"), fullpath, string(" "));

    if (!root_nh.hasParam("/AC_para_X"))
        ROS_WARN_STREAM("Parameter [/AC_para_X] not found, using default: ");
    else
        ROS_INFO_STREAM("Parameter [/AC_para_X] = ");
    ROS_INFO_STREAM(_STIFFNESS[0] << "\t" <<
      AC_para_mass[0] << "\t" <<
      AC_para_alpha[0]);
    if (!root_nh.hasParam("/AC_para_Y"))
        ROS_WARN_STREAM("Parameter [/AC_para_Y] not found, using default: ");
    else
        ROS_INFO_STREAM("Parameter [/AC_para_Y] = ");
    ROS_INFO_STREAM(_STIFFNESS[1] << "\t" <<
      AC_para_mass[1] << "\t" <<
      AC_para_alpha[1]);
    if (!root_nh.hasParam("/AC_para_Z"))
        ROS_WARN_STREAM("Parameter [/AC_para_Z] not found, using default: ");
    else
        ROS_INFO_STREAM("Parameter [/AC_para_Z] = ");
    ROS_INFO_STREAM(_STIFFNESS[2] << "\t" <<
      AC_para_mass[2] << "\t" <<
      AC_para_alpha[2]);

    if (!root_nh.hasParam("/forcecontrol_print_flag"))
        ROS_WARN_STREAM("Parameter [/forcecontrol_print_flag] not found, using default: " << _print_flag);
    else
        ROS_INFO_STREAM("Parameter [/forcecontrol_print_flag] = " << _print_flag);

    if (!root_nh.hasParam("/forcecontrol_file_path"))
        ROS_WARN_STREAM("Parameter [/forcecontrol_file_path] not found, using default: " << fullpath);
    else
        ROS_INFO_STREAM("Parameter [/forcecontrol_file_path] = " << fullpath);

    // record the transformation
    _T << T_elements[0], T_elements[1], T_elements[2],
            T_elements[3], T_elements[4], T_elements[5],
            T_elements[6], T_elements[7], T_elements[8];

    _dt = 1.0f/fHz;
    // compute the gains
    for (int i = 0; i < 3; ++i)
    {
        _COMP1_K(i)    = 1.0f/fHz/AC_para_mass[i];
        _COMP1_ZERO(i) = 0.0f;
        _COMP1_POLE(i) = 1.0f - AC_para_alpha[i]/fHz/AC_para_mass[i];
        _COMP2_K(i)    = 1000.0f/fHz;
        _COMP2_ZERO(i) = 0.0f;
        _COMP2_POLE(i) = 1.0f;
    }

    // open file
    if (_print_flag)
    {
        _file.open(fullpath);
        if (_file.is_open())
            ROS_INFO_STREAM("[ForceControlController] file opened successfully."
                    << endl);
        else
            ROS_ERROR_STREAM("[ForceControlController] Failed to open file."
                    << endl);
    }
    return true;
}

void ForceControlController::setPose(const float *pose)
{
    UT::copyArray(pose, _pose_user_input, 7);
}

void ForceControlController::setForce(const float *force)
{
    _wrench_Tr_set(0) = force[0];
    _wrench_Tr_set(1) = force[1];
    _wrench_Tr_set(2) = force[2];
    _wrench_Tr_set(3) = force[3];
    _wrench_Tr_set(4) = force[4];
    _wrench_Tr_set(5) = force[5];
}

/*
 *
    force control law
 *
 */
bool ForceControlController::update(const ros::Time& time, const ros::Duration& period)
{
    float pose_fb[7];
    float wrench_fb[6];
    if (!_hw->getState(pose_fb, wrench_fb)) return false;

    // ----------------------------------------
    //  Update offset
    // ----------------------------------------

    Matrix6f m_force_selection    = _v_force_selection.asDiagonal();
    Matrix6f m_velocity_selection = _v_velocity_selection.asDiagonal();

    // ----------------------------------------
    //  Compute Forces in Generalized space
    // ----------------------------------------
    /*  Spring forces  (using stiffness)   */
    // Pose Error
    Matrix4f SE3_WTSet;
    SE3_WTSet = pose2SE3(_pose_user_input);

    Matrix4f SE3_WSo;
    SE3_WSo = _SE3_WToffset * SE3_WTSet;

    Matrix4f SE3_WT_fb, SE3_TSo;
    SE3_WT_fb = pose2SE3(pose_fb);
    SE3_TSo   = SE3Inv(SE3_WT_fb)*SE3_WSo; // aka SE3_S_err

    Vector6f spt_TSo;
    spt_TSo = SE32spt(SE3_TSo);

    // elastic wrench
    Vector6f wrench_T_spring;
    wrench_T_spring = Jac0_*_ToolStiffnessMatrix*spt_TSo;

    /*  Force feedback  */
    // note: the minus sign here make the physical meaning of the force correct:
    //      the force being acted on the environment from the robot
    Vector6f wrench_T_fb; // force feedback measured in tool frame
    for (int i = 0; i < 6; ++i) wrench_T_fb(i) = - wrench_fb[i];

    /* velocity */
    Matrix6f Adj_WT = SE32Adj(SE3_WT_fb);
    Matrix6f Adj_TW = SE32Adj(SE3Inv(SE3_WT_fb));
    Vector6f v_Tr =  _T * Adj_TW * _v_W;

    /* transformation from Tool wrench to
            transformed space  */
    Vector6f wrench_Tr_spring, wrench_Tr_fb;
    wrench_Tr_spring = _T*wrench_T_spring;
    wrench_Tr_fb     = _T*wrench_T_fb;

    /*  Force error, PID force control */
    Eigen::Vector6f wrench_Tr_Err = _wrench_Tr_set - wrench_Tr_fb;
    _wrench_Tr_Err_I += wrench_Tr_Err;
    // todo: compute the limit element-wisely based on -T
    truncate3f(&_wrench_Tr_Err_I, _FC_I_Limit, -_FC_I_Limit);
    _wrench_Tr_Err = wrench_Tr_Err;

    Vector6f wrench_Tr_PID;
    wrench_Tr_PID =  _kForceControlPGain*wrench_Tr_Err
            + _kForceControlIGain*_wrench_Tr_Err_I
            + _kForceControlDGain*(wrench_Tr_Err - _wrench_Tr_Err);

    Vector6f wrench_Tr_damping;
    wrench_Tr_damping = - _damping_coef.asDiagonal()*v_Tr;

    Vector6f wrench_Tr_All = m_force_selection *
            (wrench_Tr_spring + wrench_Tr_Err + wrench_Tr_PID + wrench_Tr_damping);

    // {
    //     using namespace std;
    //     // cout << "_T: " << _T << endl;
    //     cout << ",f_TFeedback: " << f_TFeedback(0) << "|"
    //             << f_TFeedback(1) << "|"
    //             << f_TFeedback(2);
    //     cout << ",_wrench_Tr_set: " << _wrench_Tr_set(0) << "|"
    //             << _wrench_Tr_set(1) << "|"
    //             << _wrench_Tr_set(2);
    //     cout << ",wrench_Tr_All: " << wrench_Tr_All(0) << "|"
    //             << wrench_Tr_All(1) << "|"
    //             << wrench_Tr_All(2) << endl;
    // //     // cout << "wrench_Tr_Err: " << wrench_Tr_Err << endl;
    // //     // getchar();
    // }

    // ----------------------------------------
    //  force to velocity
    // ----------------------------------------

    /* Newton's Law */
    //  Axes are no longer independent when we take
    //      rotation in to consideration.
    //  Newton's Law in body (Tool) frame:
    //      W=M*vd
    //          W: body wrench
    //          M: Inertia matrix in body frame
    //          vd: body velocity time derivative
    //  Newton's law in transformed space
    //      TW=TMTinv Tvd
    //      W_Tr = TMTinv vd_Tr
    // ----------------------------------------
    Matrix6f Tinv = _T.inverse();
    Vector6f vd_Tr = (_T*_ToolInertiaMatrix*Tinv).fullPivLu().solve(wrench_Tr_All);
    // integration
    // right now the velocity vector _vd_Tr only contains force command
    v_Tr += _dt * vd_Tr;
    v_Tr = m_force_selection * v_Tr;

    /* Velocity command */
    Vector6f v_T_command = Jac0_inv * spt_TSo / _dt;
    v_Tr += m_velocity_selection*_T*v_T_command;
    _v_W = Adj_WT * Tinv * v_Tr;

    // ----------------------------------------
    //  velocity to pose
    // ----------------------------------------
    wedge SE3_WT_fb

    Vector6f t_T_set = twist _dt * td_T;
    Matrix6f SE3_T_set = Twist2SE3(t_T_set);

    Vector6f t_WT = SE32Twist(SE3_WT_fb);
    Matrix4f SE3_WT_command;
    SE3_WT_command = SE3_WT_fb + wedge6(_v_W)*SE3_WT_fb*_dt;
    SE32Pose(SE3_WT_command, _pose_sent_to_robot);

    Clock::time_point timenow_clock = Clock::now();
    double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(
            timenow_clock - _time0).count())/1e6; // milli second

    // displayStates();
    // std::cout << "Press ENTER to continue..." << std::endl;
    // getchar();

    _hw->setControl(_pose_sent_to_robot);

    // cout << "[ForceControlController] Update at "  << timenow << endl;
    if(_print_flag)
    {
        _file << timenow << " ";
        UT::stream_array_in(_file, _pose_user_input, 7);
        UT::stream_array_in(_file, pose_fb, 7);
        UT::stream_array_in(_file, wrench_fb, 6);
        stream_array_in3f(_file, _wrench_Tr_All_old);
        stream_array_in3f(_file, _v_T);
        stream_array_in3f(_file, p_T);
        UT::stream_array_in(_file, _pose_sent_to_robot, 7);
        _file << endl;
    }

    return true;
}

void ForceControlController::updateAxis(Matrix6f T, int n_af)
{
    Vector6f p_WOffset, p_WOffsetAll; _SE3_WToffset
    p_WOffset(0) = _pose_offset[0];
    p_WOffset(1) = _pose_offset[1];
    p_WOffset(2) = _pose_offset[2];
    p_WOffsetAll(0) = _pose_user_input[0] - _pose_sent_to_robot[0];
    p_WOffsetAll(1) = _pose_user_input[1] - _pose_sent_to_robot[1];
    p_WOffsetAll(2) = _pose_user_input[2] - _pose_sent_to_robot[2];

    switch (n_af) {
        case 0:
            _v_force_selection << 0, 0, 0;
            _v_velocity_selection << 1, 1, 1;
            break;
        case 1:
            _v_force_selection << 1, 0, 0;
            _v_velocity_selection << 0, 1, 1;
            break;
        case 2:
            _v_force_selection << 1, 1, 0;
            _v_velocity_selection << 0, 0, 1;
            break;
        case 3:
            _v_force_selection << 1, 1, 1;
            _v_velocity_selection << 0, 0, 0;
            break;
        default:
            std::cout << "[ForceControlController::updateAxis] wrong value for n_af: "
                    << n_af << std::endl;
            exit(1);
    }
    Matrix3f m_force_selection    = _v_force_selection.asDiagonal();
    Matrix3f m_velocity_selection = _v_velocity_selection.asDiagonal();

    Vector3f p_TOffset = m_force_selection*T*p_WOffset
            + m_velocity_selection*T*p_WOffsetAll;
    p_WOffset = T.inverse()*p_TOffset;
    _pose_offset[0] = p_WOffset(0);
    _pose_offset[1] = p_WOffset(1);
    _pose_offset[2] = p_WOffset(2);

    // project these into force space
    Matrix3f T_old_inv = _T.inverse();
    _wrench_Tr_All_old = m_force_selection*T*T_old_inv*_wrench_Tr_All_old;
    _v_T        = m_force_selection*T*T_old_inv*_v_T;
    _v_T_old    = m_force_selection*T*T_old_inv*_v_T_old;

    _wrench_Tr_Err_I   = m_force_selection*T*T_old_inv*_wrench_Tr_Err_I;
    _wrench_Tr_Err     = m_force_selection*T*T_old_inv*_wrench_Tr_Err;

    _T = T;
    _n_af = n_af;
}

// reset everytime you start from complete stop.
void ForceControlController::reset()
{
    _hw->getPose(_pose_sent_to_robot);
    for (int ax = 0; ax < 3; ++ax)
    {
        _pose_offset[ax]  = 0;
        _p_W(ax) = _pose_sent_to_robot[ax];
    }

    _wrench_Tr_Err     = Vector3f::Zero();
    _wrench_Tr_Err_I   = Vector3f::Zero();
    _wrench_Tr_set     = Vector3f::Zero();
    _wrench_Tr_All_old = Vector3f::Zero();
    _v_T        = Vector3f::Zero();
    _v_T_old    = Vector3f::Zero();

    UT::copyArray(_pose_sent_to_robot, _pose_user_input, 7);
}

void ForceControlController::displayStates()
{
    std::cout << "================= States ================== " << std::endl;
    std::cout << "_pose_user_input: " << _pose_user_input[0] << ", "
            << _pose_user_input[1] << ", " << _pose_user_input[2]
            << std::endl;
    std::cout << "pose offset: " << _pose_offset[0] << ", " << _pose_offset[1]
            << ", " << _pose_offset[2] << std::endl;
    std::cout << "pose sent to robot: " << _pose_sent_to_robot[0] << ", "
            << _pose_sent_to_robot[1] << ", " << _pose_sent_to_robot[2]
            << std::endl;
    std::cout << "_v_force_selection: " << _v_force_selection[0] << ", "
            << _v_force_selection[1] << ", " << _v_force_selection[2]
            << std::endl;
    std::cout << "_v_velocity_selection: " << _v_velocity_selection[0] << ", "
            << _v_velocity_selection[1] << ", " << _v_velocity_selection[2]
            << std::endl;
    std::cout << "_wrench_Tr_Err: " << _wrench_Tr_Err[0] << ", "
            << _wrench_Tr_Err[1] << ", " << _wrench_Tr_Err[2]
            << std::endl;
    std::cout << "_wrench_Tr_Err_I: " << _wrench_Tr_Err_I[0] << ", "
            << _wrench_Tr_Err_I[1] << ", " << _wrench_Tr_Err_I[2]
            << std::endl;
    std::cout << "_wrench_Tr_All_old: " << _wrench_Tr_All_old[0] << ", "
            << _wrench_Tr_All_old[1] << ", " << _wrench_Tr_All_old[2]
            << std::endl;
    std::cout << "_v_T: " << _v_T[0] << ", "
            << _v_T[1] << ", " << _v_T[2]
            << std::endl;
    std::cout << "_v_T_old: " << _v_T_old[0] << ", "
            << _v_T_old[1] << ", " << _v_T_old[2]
            << std::endl;
    std::cout << "_p_W: " << _p_W[0] << ", "
            << _p_W[1] << ", " << _p_W[2]
            << std::endl;
}