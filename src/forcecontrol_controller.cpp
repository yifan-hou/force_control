#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>
#include <string>

#include <Eigen/QR>

#include <forcecontrol/utilities.h>


typedef std::chrono::high_resolution_clock Clock;
// typedef Eigen::Vector3f Vector3f;
// typedef Eigen::Matrix3f Matrix3f;
// typedef Eigen::Matrix4f Matrix4f;
// typedef Eigen::MatrixXf MatrixXf;
// typedef Eigen::Matrix<float, 6, 1> Vector6f;
// typedef Eigen::Matrix<float, 6, 6> Matrix6f;

using namespace UT;

ForceControlController::ForceControlController()
{
    _pose_user_input      = new float[7];
    _wrench_Tr_set        = Vector6f::Zero();
    _Tr                   = Matrix6f::Identity();
    _Tr_inv               = Matrix6f::Identity();
    _m_force_selection    = Matrix6f::Zero();
    _m_velocity_selection = Matrix6f::Identity();

    _pose_sent_to_robot = new float[7];
    _v_W                = Vector6f::Zero();
    _wrench_Tr_Err      = Vector6f::Zero();
    _wrench_Tr_Err_I    = Vector6f::Zero();
    _SE3_WT_old         = Matrix4f::Identity();
    _SE3_WToffset       = Matrix4f::Identity();
}

ForceControlController::~ForceControlController()
{
    delete [] _pose_user_input;
    delete [] _pose_sent_to_robot;

    if (_print_flag)
        _file.close();
}

bool ForceControlController::init(ros::NodeHandle& root_nh, ForceControlHardware* hw, std::chrono::high_resolution_clock::time_point time0)
{
    _hw    = hw;
    _time0 = time0;

    /* use current state to initialize commands */
    float wrench[6];
    _hw->getState(_pose_user_input, wrench);
    stream_array_in(cout, _pose_user_input, 7);
    copyArray(_pose_user_input, _pose_sent_to_robot, 7);
    _SE3_WT_old = pose2SE3(_pose_user_input);

    /* read controller parameters from parameter server */

    // control rate
    float fHz;
    root_nh.param(string("/main_loop_rate"), fHz, 500.0f);
    if (!root_nh.hasParam("/main_loop_rate"))
        ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << fHz);
    else
        ROS_INFO_STREAM("Parameter [/main_loop_rate] = " << fHz);

    _dt = 1.0f/fHz;

    // Spring-mass-damper coefficients
    std::vector<float> Stiffness_matrix_diag_elements,
            Inertia_matrix_diag_elements,
            Damping_matrix_diag_elements;
    root_nh.getParam("/Stiffness_matrix_diag_elements",
            Stiffness_matrix_diag_elements);
    root_nh.getParam("/Inertia_matrix_diag_elements",
            Inertia_matrix_diag_elements);
    root_nh.getParam("/Damping_matrix_diag_elements",
            Damping_matrix_diag_elements);

    if (!root_nh.hasParam("/Stiffness_matrix_diag_elements")) {
        ROS_ERROR_STREAM("Parameter [/Stiffness_matrix_diag_elements] not found");
        return false;
    }
    if (!root_nh.hasParam("/Inertia_matrix_diag_elements")) {
        ROS_ERROR_STREAM("Parameter [/Inertia_matrix_diag_elements] not found");
        return false;
    }
    if (!root_nh.hasParam("/Damping_matrix_diag_elements")) {
        ROS_ERROR_STREAM("Parameter [/Damping_matrix_diag_elements] not found");
        return false;
    }
    _ToolStiffnessMatrix = Vector6f(Stiffness_matrix_diag_elements.data()).asDiagonal();
    _damping_coef        = Vector6f(Damping_matrix_diag_elements.data()).asDiagonal();
    _ToolInertiaMatrix   = Vector6f(Inertia_matrix_diag_elements.data()).asDiagonal();

    // force control gains
    root_nh.param(string("/FC_gains/PGain"), _kForceControlPGain, 1.0f);
    root_nh.param(string("/FC_gains/IGain"), _kForceControlIGain, 0.0f);
    root_nh.param(string("/FC_gains/DGain"), _kForceControlDGain, 0.0f);
    root_nh.param(string("/FC_I_Limit"), _FC_I_Limit, 10.0f);
    if (!root_nh.hasParam("/FC_gains")) {
        ROS_WARN_STREAM("Parameter [/FC_gains] not found, using default.");
    } else {
        ROS_INFO_STREAM("Parameter [/FC_gains/PGain] = " << _kForceControlPGain);
        ROS_INFO_STREAM("Parameter [/FC_gains/IGain] = " << _kForceControlIGain);
        ROS_INFO_STREAM("Parameter [/FC_gains/DGain] = " << _kForceControlDGain);
    }

    // printing
    string fullpath;
    root_nh.param(string("/forcecontrol_print_flag"), _print_flag, false);
    root_nh.param(string("/forcecontrol_file_path"), fullpath, string(" "));

    if (!root_nh.hasParam("/forcecontrol_print_flag"))
        ROS_WARN_STREAM("Parameter [/forcecontrol_print_flag] not found, using default: " << _print_flag);
    else
        ROS_INFO_STREAM("Parameter [/forcecontrol_print_flag] = " << _print_flag);

    if (!root_nh.hasParam("/forcecontrol_file_path"))
        ROS_WARN_STREAM("Parameter [/forcecontrol_file_path] not found, using default: " << fullpath);
    else
        ROS_INFO_STREAM("Parameter [/forcecontrol_file_path] = " << fullpath);

    if (_print_flag) {
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
        Frames/spaces:
            W: world frame
            T: current tool frame
            So: set tool frame with offset
            Tf: transformed generalized space
        Quantities:
            SE3: 4x4 homogeneous coordinates
            se3: 6x1 twist coordinate of SE3
            spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for rotation
            td: 6x1 time derivative of twist.
            v: 6x1 velocity, either spatial or body
            wrench: 6x1 wrench. Makes work with body velocity
 *
 */
bool ForceControlController::update(const ros::Time& time, const ros::Duration& period)
{
    float pose_fb[7];
    float wrench_fb[6];
    if (!_hw->getState(pose_fb, wrench_fb)) return false;

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

    // Jac0 * body velocity = spt time derivative
    Matrix6f Jac0, Jac0_inv;
    Jac0_inv = JacobianSpt2BodyV(SE3_WT_fb.block<3,3>(0,0));
    Jac0 = Jac0_inv.inverse();

    // elastic wrench
    Vector6f wrench_T_spring;
    wrench_T_spring = Jac0*_ToolStiffnessMatrix*spt_TSo;

    /*  Force feedback  */
    // note: the minus sign here make the physical meaning of the force correct:
    //      the force being acted on the environment from the robot
    Vector6f wrench_T_fb; // force feedback measured in tool frame
    for (int i = 0; i < 6; ++i) wrench_T_fb(i) = - wrench_fb[i];

    /* velocity */
    Matrix6f Adj_WT = SE32Adj(SE3_WT_fb);
    Matrix6f Adj_TW = SE32Adj(SE3Inv(SE3_WT_fb));
    Vector6f v_Tr =  _Tr * Adj_TW * _v_W;

    /* transformation from Tool wrench to
            transformed space  */
    Vector6f wrench_Tr_spring, wrench_Tr_fb;
    wrench_Tr_spring = _Tr*wrench_T_spring;
    wrench_Tr_fb     = _Tr*wrench_T_fb;

    /*  Force error, PID force control */
    Vector6f wrench_Tr_Err = _wrench_Tr_set - wrench_Tr_fb;
    _wrench_Tr_Err_I += wrench_Tr_Err;
    // todo: compute the limit element-wisely based on -T
    truncate6f(&_wrench_Tr_Err_I, _FC_I_Limit, -_FC_I_Limit);

    Vector6f wrench_Tr_PID;
    wrench_Tr_PID =  _kForceControlPGain*wrench_Tr_Err
            + _kForceControlIGain*_wrench_Tr_Err_I
            + _kForceControlDGain*(wrench_Tr_Err - _wrench_Tr_Err);
    _wrench_Tr_Err = wrench_Tr_Err;

    Vector6f wrench_Tr_damping;
    wrench_Tr_damping = - _damping_coef*v_Tr;

    Vector6f wrench_Tr_All = _m_force_selection *
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
    Matrix6f Tinv = _Tr.inverse();
    Vector6f vd_Tr = (_Tr*_ToolInertiaMatrix*Tinv).fullPivLu().solve(wrench_Tr_All);
    // integration
    // right now the velocity vector _vd_Tr only contains force command
    v_Tr += _dt * vd_Tr;
    v_Tr = _m_force_selection * v_Tr;

    /* Velocity command */
    Vector6f v_T_command = Jac0_inv * spt_TSo / _dt;
    v_Tr += _m_velocity_selection*_Tr*v_T_command;
    _v_W = Adj_WT * Tinv * v_Tr;

    // ----------------------------------------
    //  velocity to pose
    // ----------------------------------------
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
        stream_array_in(_file, _pose_user_input, 7);
        stream_array_in(_file, pose_fb, 7);
        stream_array_in(_file, wrench_fb, 6);
        stream_array_in6f(_file, wrench_Tr_All);
        stream_array_in(_file, _pose_sent_to_robot, 7);
        _file << endl;
    }

    return true;
}

// After axis update, the goal pose with offset should not have error in
// velocity controlled axes of the new axes. To satisfy this requirement,
// we need to change _SE3_WToffset accordingly
void ForceControlController::updateAxis(const Matrix6f &Tr, int n_af)
{
    Matrix4f SE3_WTSet;
    SE3_WTSet = pose2SE3(_pose_user_input);

    Matrix4f SE3_WSo;
    SE3_WSo = _SE3_WToffset * SE3_WTSet;

    Matrix4f SE3_WT_fb, SE3_TSo;
    SE3_WT_fb = pose2SE3(_pose_sent_to_robot);
    SE3_TSo   = SE3Inv(SE3_WT_fb)*SE3_WSo; // aka SE3_S_err

    Vector6f spt_TSo;
    spt_TSo = SE32spt(SE3_TSo);

    Vector6f v_force_selection, v_velocity_selection;
    v_force_selection << 0, 0, 0, 0, 0, 0;
    v_velocity_selection << 1, 1, 1, 1, 1, 1;
    for (int i = 0; i < n_af; ++i) {
        v_force_selection(i) = 1;
        v_velocity_selection(i) = 0;
    }

    _m_force_selection    = v_force_selection.asDiagonal();
    _m_velocity_selection = v_velocity_selection.asDiagonal();

    Matrix6f Jac0;
    Jac0 = JacobianSpt2BodyV(SE3_WT_fb.block<3,3>(0,0)).inverse();

    MatrixXf m_anni = _m_velocity_selection*Tr*Jac0;
    Vector6f spt_TSo_new = (Matrix6f::Identity() -
            pseudoInverse(m_anni, 1e-6)*m_anni)*spt_TSo;
    // update offset
    _SE3_WToffset = SE3_WT_fb*spt2SE3(spt_TSo_new)*SE3Inv(SE3_WTSet);

    // project these into force space
    Matrix6f Adj_WT_old = SE32Adj(_SE3_WT_old);
    Matrix6f Adj_TW = SE32Adj(SE3Inv(SE3_WT_fb));

    _wrench_Tr_Err_I = _m_force_selection*Tr*Adj_TW*Adj_WT_old*_Tr_inv*_wrench_Tr_Err_I;
    _wrench_Tr_Err   = _m_force_selection*Tr*Adj_TW*Adj_WT_old*_Tr_inv*_wrench_Tr_Err;

    _SE3_WT_old = SE3_WT_fb;
    _Tr         = Tr;
    _Tr_inv     = _Tr.inverse();
}

// reset everytime you start from complete stop.
// clear up internal states
void ForceControlController::reset()
{
    Eigen::Matrix<float, 6, 1> _wrench_Tr_Err;
    Eigen::Matrix<float, 6, 1> _wrench_Tr_Err_I;


    _hw->getPose(_pose_sent_to_robot);
    _SE3_WT_old = pose2SE3(_pose_sent_to_robot);
    _SE3_WToffset = Matrix4f::Identity();
    _v_W = Vector6f::Zero();
    _wrench_Tr_Err = Vector6f::Zero();
    _wrench_Tr_Err_I = Vector6f::Zero();
}

void ForceControlController::displayStates()
{
    // std::cout << "================= States ================== " << std::endl;
    // std::cout << "_pose_user_input: " << _pose_user_input[0] << ", "
    //         << _pose_user_input[1] << ", " << _pose_user_input[2]
    //         << std::endl;
    // std::cout << "pose offset: " << _pose_offset[0] << ", " << _pose_offset[1]
    //         << ", " << _pose_offset[2] << std::endl;
    // std::cout << "pose sent to robot: " << _pose_sent_to_robot[0] << ", "
    //         << _pose_sent_to_robot[1] << ", " << _pose_sent_to_robot[2]
    //         << std::endl;
    // std::cout << "_v_force_selection: " << _v_force_selection[0] << ", "
    //         << _v_force_selection[1] << ", " << _v_force_selection[2]
    //         << std::endl;
    // std::cout << "_v_velocity_selection: " << _v_velocity_selection[0] << ", "
    //         << _v_velocity_selection[1] << ", " << _v_velocity_selection[2]
    //         << std::endl;
    // std::cout << "_wrench_Tr_Err: " << _wrench_Tr_Err[0] << ", "
    //         << _wrench_Tr_Err[1] << ", " << _wrench_Tr_Err[2]
    //         << std::endl;
    // std::cout << "_wrench_Tr_Err_I: " << _wrench_Tr_Err_I[0] << ", "
    //         << _wrench_Tr_Err_I[1] << ", " << _wrench_Tr_Err_I[2]
    //         << std::endl;
    // std::cout << "_v_T: " << _v_T[0] << ", "
    //         << _v_T[1] << ", " << _v_T[2]
    //         << std::endl;
    // std::cout << "_v_T_old: " << _v_T_old[0] << ", "
    //         << _v_T_old[1] << ", " << _v_T_old[2]
    //         << std::endl;
    // std::cout << "_p_W: " << _p_W[0] << ", "
    //         << _p_W[1] << ", " << _p_W[2]
    //         << std::endl;
}