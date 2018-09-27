#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>
#include <string>

#include <forcecontrol/utilities.h>


typedef std::chrono::high_resolution_clock Clock;


using namespace std;

void truncate3f(Eigen::Vector3f *v, float max, float min)
{
    for(int i=0; i<3; i++)
    {
        (*v)[i] = ((*v)[i] > max)? max:(*v)[i];
        (*v)[i] = ((*v)[i] < min)? min:(*v)[i];
    }
}

void stream_array_in3f(ostream &st, const Eigen::Vector3f array)
{
    for (int i = 0; i<3; i++)
    {
        st << array(i);
        st << "\t";
    }
}

ForceControlController::ForceControlController()
{
    _T = Eigen::Matrix3f::Identity();
    _pose_set        = new float[7];

    _pose_offset  = new float[3]{0};
    _pose_command = new float[7];

    _f_Tset = Eigen::Vector3f::Zero();
    _f_TAll_old = Eigen::Vector3f::Zero();
    _v_T = Eigen::Vector3f::Zero();
    _v_T_old = Eigen::Vector3f::Zero();
    _p_T = Eigen::Vector3f::Zero();
}

ForceControlController::~ForceControlController()
{
    delete [] _pose_set;
    delete [] _pose_offset;
    delete [] _pose_command;

    if (_print_flag)
        _file.close();
}

bool ForceControlController::init(ros::NodeHandle& root_nh, ForceControlHardware* hw, std::chrono::high_resolution_clock::time_point time0)
{
    _hw    = hw;
    _time0 = time0;

    // read set pose
    float wrench[6];
    _hw->getState(_pose_set, wrench);
    std::cout << "[ForceControlController] set pose: " << std::endl;
    UT::stream_array_in(cout, _pose_set, 7);
    std::cout << std::endl;
    std::cout << "[ForceControlController] set force: " << std::endl;
    std::cout << _f_Tset(0) << ", " << _f_Tset(1) << ", " << _f_Tset(2);
    std::cout << std::endl;

    // read controller from parameter server
    float AC_para_mass[3], AC_para_alpha[3];
    float fHz;
    std::vector<float> T_elements;
    string fullpath;
    root_nh.getParam("/Transform", T_elements);
    root_nh.param(string("/main_loop_rate"), fHz, 500.0f);
    root_nh.param(string("/force_controlled_dimensions"), _n_af, 3);
    root_nh.param(string("/AC_para_X/k"), _STIFFNESS(0), 0.0f);
    root_nh.param(string("/AC_para_Y/k"), _STIFFNESS(1), 0.0f);
    root_nh.param(string("/AC_para_Z/k"), _STIFFNESS(2), 0.0f);
    root_nh.param(string("/AC_para_X/m"),     AC_para_mass[0], 1.0f);
    root_nh.param(string("/AC_para_Y/m"),     AC_para_mass[1], 1.0f);
    root_nh.param(string("/AC_para_Z/m"),     AC_para_mass[2], 1.0f);
    root_nh.param(string("/AC_para_X/alpha"),     AC_para_alpha[0], 100.0f);
    root_nh.param(string("/AC_para_Y/alpha"),     AC_para_alpha[1], 100.0f);
    root_nh.param(string("/AC_para_Z/alpha"),     AC_para_alpha[2], 100.0f);
    root_nh.param(string("/AC_limit"),     _COMP2_LIMIT, 100.0f);
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

    if (!root_nh.hasParam("/AC_limit"))
        ROS_WARN_STREAM("Parameter [/AC_limit] not found, using default: " << _COMP2_LIMIT);
    else
        ROS_INFO_STREAM("Parameter [/AC_limit] = " << _COMP2_LIMIT);

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
    UT::copyArray(pose, _pose_set, 7);
}

void ForceControlController::setForce(const float *force)
{
    _f_Tset(0) = force[0];
    _f_Tset(1) = force[1];
    _f_Tset(2) = force[2];
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

    Eigen:Vector3f v_force_selection;
    switch (_n_af) {
        case 0:
            v_force_selection << 0, 0, 0;
            break;
        case 1:
            v_force_selection << 1, 0, 0;
            break;
        case 2:
            v_force_selection << 1, 1, 0;
            break;
        case 3:
            v_force_selection << 1, 1, 1;
            break;
        default:
            std::cout << "[ForceControlController::update] wrong value for _n_af: "
                    << _n_af << std::endl;
            exit(1);
    }
    Eigen::Matrix3f m_force_selection = v_force_selection.asDiagonal();
    // ----------------------------------------
    //  Pose error
    // ----------------------------------------
    Eigen::Vector3f v_WPoseErr;
    for (int i = 0; i < 3; ++i)
        v_WPoseErr[i] = _pose_set[i] - _pose_offset[i] - pose_fb[i];
    Eigen::Vector3f v_TPoseErr = _T * v_WPoseErr;

    // ----------------------------------------
    //  Spring forces (using stiffness)
    // ----------------------------------------
    Eigen::Vector3f f_TSpring;
    f_TSpring = _STIFFNESS.asDiagonal()*v_TPoseErr;

    // ----------------------------------------
    //  Force feedback
    // ----------------------------------------
    static Eigen::Vector3f f_local_feedback;
    static Quaternionf qn; // current orientation of end effector (also FT sensor)

    f_local_feedback(0) = wrench_fb[0];
    f_local_feedback(1) = wrench_fb[1];
    f_local_feedback(2) = wrench_fb[2];

    qn.w() = pose_fb[3];
    qn.x() = pose_fb[4];
    qn.y() = pose_fb[5];
    qn.z() = pose_fb[6];

    // transformation
    Eigen::Vector3f f_TFeedback = _T*qn._transformVector(f_local_feedback);

    // ----------------------------------------
    //  Force error, PID force control
    // ----------------------------------------
    Eigen::Vector3f f_TErr = m_force_selection*(_f_Tset - f_TFeedback);
    _f_TErr_I += f_TErr;
    truncate3f(&_f_TErr_I, _FC_I_Limit, -_FC_I_Limit);

    Eigen::Vector3f f_TPID;
    f_TPID = _f_Tset - _kForceControlPGain*f_TErr
            - _kForceControlIGain*_f_TErr_I
            + _kForceControlDGain*(f_TErr - _f_TErr);

    _f_TErr = f_TErr;
    Eigen::Vector3f f_TAll =
            m_force_selection*(f_TSpring + f_TFeedback + f_TPID);

    // ----------------------------------------
    //  Compensator 1
    // ----------------------------------------
    _v_T = _COMP1_POLE.asDiagonal()*_v_T + _COMP1_K.asDiagonal()*f_TAll
            - Eigen::Matrix3f(_COMP1_ZERO.asDiagonal())*_COMP1_K.asDiagonal()*_f_TAll_old;
    _f_TAll_old = f_TAll;

    // ----------------------------------------
    //  Compensator 2
    // ----------------------------------------
    _p_T = _COMP2_POLE.asDiagonal()*_p_T + _COMP2_K.asDiagonal()*_v_T
            - Eigen::Matrix3f(_COMP2_ZERO.asDiagonal())*_COMP2_K.asDiagonal()*_v_T_old;
    truncate3f(&_p_T, _COMP2_LIMIT, -_COMP2_LIMIT);
    _v_T_old = _v_T;

    // ----------------------------------------
    //  Pose offset
    // ----------------------------------------
    Eigen::Vector3f v_Woffset = _T.inverse()*_p_T;

    for (int i = 0; i < 3; ++i)
        _pose_command[i] = _pose_set[i] - _pose_offset[i] + v_Woffset[i];
    for (int i = 3; i < 7; ++i)
        _pose_command[i] = _pose_set[i];

    Clock::time_point timenow_clock = Clock::now();
    double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(
            timenow_clock - _time0).count())/1e6; // milli second

    _hw->setControl(_pose_command);

    // cout << "[ForceControlController] Update at "  << timenow << endl;
    if(_print_flag)
    {
        _file << timenow << " ";
        UT::stream_array_in(_file, _pose_set, 7);
        UT::stream_array_in(_file, pose_fb, 7);
        UT::stream_array_in(_file, wrench_fb, 6);
        stream_array_in3f(_file, _f_TAll_old);
        stream_array_in3f(_file, _v_T);
        stream_array_in3f(_file, _p_T);
        UT::stream_array_in(_file, _pose_command, 7);
        _file << endl;
    }

    return true;
}

void ForceControlController::updateAxis(Eigen::Matrix3f T, int n_af)
{
    Eigen::Vector3f p_WOffset, p_WOffsetAll;
    p_WOffset(0) = _pose_offset[0];
    p_WOffset(1) = _pose_offset[1];
    p_WOffset(2) = _pose_offset[2];
    p_WOffsetAll(0) = _pose_set[0] - _pose_command[0];
    p_WOffsetAll(1) = _pose_set[1] - _pose_command[1];
    p_WOffsetAll(2) = _pose_set[2] - _pose_command[2];

    Eigen::Vector3f v_force_selection;
    Eigen::Vector3f v_velocity_selection;
    switch (n_af) {
        case 0:
            v_force_selection << 0, 0, 0;
            v_velocity_selection << 1, 1, 1;
            break;
        case 1:
            v_force_selection << 1, 0, 0;
            v_velocity_selection << 0, 1, 1;
            break;
        case 2:
            v_force_selection << 1, 1, 0;
            v_velocity_selection << 0, 0, 1;
            break;
        case 3:
            v_force_selection << 1, 1, 1;
            v_velocity_selection << 0, 0, 0;
            break;
        default:
            std::cout << "[ForceControlController::updateAxis] wrong value for n_af: "
                    << n_af << std::endl;
            exit(1);
    }
    Eigen::Matrix3f m_force_selection    = v_force_selection.asDiagonal();
    Eigen::Matrix3f m_velocity_selection = v_velocity_selection.asDiagonal();

    Eigen::Vector3f p_TOffset = m_force_selection*T*p_WOffset
            + m_velocity_selection*T*p_WOffsetAll;
    p_WOffset = T.inverse()*p_TOffset;
    _pose_offset[0] = p_WOffset(0);
    _pose_offset[1] = p_WOffset(1);
    _pose_offset[2] = p_WOffset(2);

    // project these into force space
    Eigen::Matrix3f T_old_inv = _T.inverse();
    _f_TAll_old     = m_force_selection*T*T_old_inv*_f_TAll_old;
    _v_T     = m_force_selection*T*T_old_inv*_v_T;
    _v_T_old     = m_force_selection*T*T_old_inv*_v_T_old;
    _p_T     = m_force_selection*T*T_old_inv*_p_T;
    _f_TErr_I = m_force_selection*T*T_old_inv*_f_TErr_I;
    _f_TErr  = m_force_selection*T*T_old_inv*_f_TErr;

    _T = T;
    _n_af = n_af;
}

void ForceControlController::reset()
{
  for (int ax = 0; ax < 3; ++ax)    _pose_offset[ax]  = 0;
  _f_TErr   = Eigen::Vector3f::Zero();
  _f_TErr_I = Eigen::Vector3f::Zero();
  _f_Tset   = Eigen::Vector3f::Zero();
  _f_TAll_old      = Eigen::Vector3f::Zero();
  _v_T      = Eigen::Vector3f::Zero();
  _v_T_old      = Eigen::Vector3f::Zero();
  _p_T      = Eigen::Vector3f::Zero();

  _hw->getPose(_pose_command);
  UT::copyArray(_pose_command, _pose_set, 7);
}
