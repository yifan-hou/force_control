#include <forcecontrol/forcecontrol_controller.h>

#include <iostream>
#include <string>

#include <Eigen/QR>

#include <forcecontrol/utilities.h>


typedef std::chrono::high_resolution_clock Clock;

using namespace UT;


Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
      "]");

ForceControlController::ForceControlController()
{
    _pose_user_input      = new double[7];
    _wrench_Tr_set        = Vector6d::Zero();
    _Tr                   = Matrix6d::Identity();
    _Tr_inv               = Matrix6d::Identity();
    _m_force_selection    = Matrix6d::Zero();
    _m_velocity_selection = Matrix6d::Identity();

    _pose_sent_to_robot = new double[7];
    _v_W                = Vector6d::Zero();
    _wrench_T_Err      = Vector6d::Zero();
    _wrench_T_Err_I    = Vector6d::Zero();
    _SE3_WT_old         = Matrix4d::Identity();
    _SE3_WToffset       = Matrix4d::Identity();
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
    double wrench[6];
    _hw->getState(_pose_user_input, wrench);
    stream_array_in(cout, _pose_user_input, 7);
    copyArray(_pose_user_input, _pose_sent_to_robot, 7);
    _SE3_WT_old = posemm2SE3(_pose_user_input);

    /* read controller parameters from parameter server */

    // control rate
    double fHz;
    root_nh.param(string("/main_loop_rate"), fHz, 500.0);
    if (!root_nh.hasParam("/main_loop_rate"))
        ROS_WARN_STREAM("Parameter [/main_loop_rate] not found, using default: " << fHz);
    else
        ROS_INFO_STREAM("Parameter [/main_loop_rate] = " << fHz);

    _dt = 1.0/fHz;

    // Spring-mass-damper coefficients
    std::vector<double> Stiffness_matrix_diag_elements,
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
    _ToolStiffnessMatrix = Vector6d(Stiffness_matrix_diag_elements.data()).asDiagonal();
    _ToolDamping_coef    = Vector6d(Damping_matrix_diag_elements.data()).asDiagonal();
    _ToolInertiaMatrix   = Vector6d(Inertia_matrix_diag_elements.data()).asDiagonal();

    // force control gains
    std::vector<double> FC_I_Limit_T_6D_elements;

    root_nh.param(string("/FC_gains/PGainT"), _kForceControlPGainTran, 0.0);
    root_nh.param(string("/FC_gains/IGainT"), _kForceControlIGainTran, 0.0);
    root_nh.param(string("/FC_gains/DGainT"), _kForceControlDGainTran, 0.0);
    root_nh.param(string("/FC_gains/PGainR"), _kForceControlPGainRot, 0.0);
    root_nh.param(string("/FC_gains/IGainR"), _kForceControlIGainRot, 0.0);
    root_nh.param(string("/FC_gains/DGainR"), _kForceControlDGainRot, 0.0);
    root_nh.getParam("/FC_I_Limit_T_6D", FC_I_Limit_T_6D_elements);
    if (!root_nh.hasParam("/FC_gains")) {
        ROS_WARN_STREAM("Parameter [/FC_gains] not found, using default.");
    } else {
        ROS_INFO_STREAM("Parameter [/FC_gains/PGainT] = " << _kForceControlPGainTran);
        ROS_INFO_STREAM("Parameter [/FC_gains/IGainT] = " << _kForceControlIGainTran);
        ROS_INFO_STREAM("Parameter [/FC_gains/DGainT] = " << _kForceControlDGainTran);
        ROS_INFO_STREAM("Parameter [/FC_gains/PGainR] = " << _kForceControlPGainRot);
        ROS_INFO_STREAM("Parameter [/FC_gains/IGainR] = " << _kForceControlIGainRot);
        ROS_INFO_STREAM("Parameter [/FC_gains/DGainR] = " << _kForceControlDGainRot);
    }
    if (!root_nh.hasParam("/FC_I_Limit_T_6D")) {
        ROS_ERROR_STREAM("Parameter [/FC_I_Limit_T_6D] not found");
        return false;
    }
    _FC_I_limit_T_6D = Vector6d(FC_I_Limit_T_6D_elements.data());

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

void ForceControlController::setPose(const double *pose)
{
    UT::copyArray(pose, _pose_user_input, 7);
}

void ForceControlController::setForce(const double *force)
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
    double pose_fb[7];
    double wrench_fb[6];
    bool is_safe = _hw->getState(pose_fb, wrench_fb);
    // ----------------------------------------
    //  Compute Forces in Generalized space
    // ----------------------------------------
    /*  Spring forces  (using stiffness)   */
    // Pose Error
    Matrix4d SE3_WTSet;
    SE3_WTSet = posemm2SE3(_pose_user_input);

    Matrix4d SE3_WSo;
    SE3_WSo = _SE3_WToffset * SE3_WTSet;

    Matrix4d SE3_WT_fb, SE3_TSo;
    SE3_WT_fb = posemm2SE3(pose_fb);
    SE3_TSo   = SE3Inv(SE3_WT_fb)*SE3_WSo; // aka SE3_S_err

    Vector6d spt_TSo;
    spt_TSo = SE32spt(SE3_TSo);

    // Jac0 * body velocity = spt time derivative
    Matrix6d Jac0, Jac0_inv;
    Jac0_inv = JacobianSpt2BodyV(SE3_WT_fb.block<3,3>(0,0));
    Jac0 = Jac0_inv.inverse();

    // elastic wrench
    Vector6d wrench_T_spring;
    wrench_T_spring = Jac0*_ToolStiffnessMatrix*spt_TSo;

    /*  Force feedback  */
    // note: the minus sign here make the physical meaning of the force correct:
    //      the force being acted on the environment from the robot
    Vector6d wrench_T_fb; // force feedback measured in tool frame
    for (int i = 0; i < 6; ++i) wrench_T_fb(i) = - wrench_fb[i];

    /* velocity */
    Matrix6d Adj_WT, Adj_TW;
    Adj_WT = SE32Adj(SE3_WT_fb);
    Adj_TW = SE32Adj(SE3Inv(SE3_WT_fb));
    Vector6d v_T, v_Tr;
    v_T  =  Adj_TW * _v_W;
    v_Tr =  _Tr * v_T;

    /* transformation from Tool wrench to
            transformed space  */
    Vector6d wrench_Tr_spring, wrench_Tr_fb;
    wrench_Tr_spring = _Tr*wrench_T_spring;

    /*  Force error, PID force control */
    Vector6d wrench_T_Set, wrench_T_Err;
    wrench_T_Set = _Tr_inv*_wrench_Tr_set;
    wrench_T_Err = wrench_T_Set - wrench_T_fb;
    _wrench_T_Err_I += wrench_T_Err;
    truncate6d(&_wrench_T_Err_I, -_FC_I_limit_T_6D, _FC_I_limit_T_6D);

    Vector6d wrench_T_PID;
    wrench_T_PID.head(3) = _kForceControlPGainTran*wrench_T_Err.head(3)
            + _kForceControlIGainTran*_wrench_T_Err_I.head(3)
            + _kForceControlDGainTran*(wrench_T_Err.head(3) - _wrench_T_Err.head(3));
    wrench_T_PID.tail(3) = _kForceControlPGainRot*wrench_T_Err.tail(3)
            + _kForceControlIGainRot*_wrench_T_Err_I.tail(3)
            + _kForceControlDGainRot*(wrench_T_Err.tail(3) - _wrench_T_Err.tail(3));
    Vector6d wrench_Tr_PID;
    wrench_Tr_PID = _Tr * wrench_T_PID;
    _wrench_T_Err = wrench_T_Err;
    Vector6d wrench_Tr_Err;
    wrench_Tr_Err = _Tr*wrench_T_Err;

    Vector6d wrench_Tr_damping;
    wrench_Tr_damping = - _Tr*_ToolDamping_coef*v_T;

    Vector6d wrench_Tr_All;
    wrench_Tr_All = _m_force_selection *
            (wrench_Tr_spring + wrench_Tr_Err + wrench_Tr_PID + wrench_Tr_damping);

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
    Matrix6d Tinv;
    Tinv = _Tr.inverse();
    Vector6d vd_Tr;
    vd_Tr = (_Tr*_ToolInertiaMatrix*Tinv).fullPivLu().solve(wrench_Tr_All);

    // integration
    // right now the velocity vector _vd_Tr only contains force command
    v_Tr += _dt * vd_Tr;
    v_Tr = _m_force_selection * v_Tr;

    /* Velocity command */
    Vector6d v_T_command;
    v_T_command = Jac0_inv * spt_TSo / _dt;
    v_Tr += _m_velocity_selection*_Tr*v_T_command;
    _v_W = Adj_WT * Tinv * v_Tr;

    // ----------------------------------------
    //  velocity to pose
    // ----------------------------------------
    Matrix4d SE3_WT_command;
    SE3_WT_command = SE3_WT_fb + wedge6(_v_W)*SE3_WT_fb*_dt;
    SE32Posemm(SE3_WT_command, _pose_sent_to_robot);

    Clock::time_point timenow_clock = Clock::now();
    double timenow = double(std::chrono::duration_cast<std::chrono::nanoseconds>(
            timenow_clock - _time0).count())/1e6; // milli second

    if (std::isnan(_pose_sent_to_robot[0])) {
        std::cout << "==================== Temp variables: =====================\n";
        cout << "pose_fb: ";
        stream_array_in(cout, pose_fb, 7);
        cout << "\n_pose_sent_to_robot: ";
        stream_array_in(cout, _pose_sent_to_robot, 7);
        std::cout << "\nwrench_Tr_spring: \n" << wrench_Tr_spring.format(MatlabFmt) << endl;
        // std::cout << "wrench_T_fb: \n" << wrench_T_fb.format(MatlabFmt) << endl;
        // std::cout << "wrench_Tr_fb: \n" << wrench_Tr_fb.format(MatlabFmt) << endl;
        // std::cout << "_wrench_Tr_set: \n" << _wrench_Tr_set.format(MatlabFmt) << endl;
        std::cout << "wrench_T_PID: \n" << wrench_T_PID.format(MatlabFmt) << endl;
        std::cout << "wrench_Tr_damping: \n" << wrench_Tr_damping.format(MatlabFmt) << endl;
        std::cout << "wrench_Tr_Err: \n" << wrench_Tr_Err.format(MatlabFmt) << endl;
        std::cout << "wrench_Tr_All: \n" << wrench_Tr_All.format(MatlabFmt) << endl;
        std::cout << "vd_Tr: \n" << vd_Tr.format(MatlabFmt) << endl;
        std::cout << "v_Tr: \n" << v_Tr.format(MatlabFmt) << endl;
        std::cout << "v_T_command: \n" << v_T_command.format(MatlabFmt) << endl;
        std::cout << "_v_W: \n" << _v_W.format(MatlabFmt) << endl;
        std::cout << "SE3_WT_fb: \n" << SE3_WT_fb.format(MatlabFmt) << endl;
        std::cout << "SE3_WT_command: \n" << SE3_WT_command.format(MatlabFmt) << endl;
        displayStates();
        std::cout << "Press ENTER to continue..." << std::endl;
        getchar();
    }

    _hw->setControl(_pose_sent_to_robot);

    // cout << "[ForceControlController] Update at "  << timenow << endl;
    if(_print_flag)
    {
        _file << timenow << " ";
        stream_array_in(_file, _pose_user_input, 7);
        stream_array_in(_file, pose_fb, 7);
        stream_array_in(_file, wrench_fb, 6);
        stream_array_in6d(_file, wrench_Tr_All);
        stream_array_in(_file, _pose_sent_to_robot, 7);
        _file << endl;
    }
    return is_safe;
}

// After axis update, the goal pose with offset should not have error in
// velocity controlled axes of the new axes. To satisfy this requirement,
// we need to change _SE3_WToffset accordingly
void ForceControlController::updateAxis(const Matrix6d &Tr, int n_af)
{
    Matrix4d SE3_WTSet;
    SE3_WTSet = posemm2SE3(_pose_user_input);

    Matrix4d SE3_WSo;
    SE3_WSo = _SE3_WToffset * SE3_WTSet;

    Matrix4d SE3_WT_fb, SE3_TSo;
    SE3_WT_fb = posemm2SE3(_pose_sent_to_robot);
    SE3_TSo   = SE3Inv(SE3_WT_fb)*SE3_WSo; // aka SE3_S_err

    Vector6d spt_TSo;
    spt_TSo = SE32spt(SE3_TSo);

    Vector6d v_force_selection, v_velocity_selection;
    v_force_selection << 0, 0, 0, 0, 0, 0;
    v_velocity_selection << 1, 1, 1, 1, 1, 1;
    for (int i = 0; i < n_af; ++i) {
        v_force_selection(i) = 1;
        v_velocity_selection(i) = 0;
    }

    _m_force_selection    = v_force_selection.asDiagonal();
    _m_velocity_selection = v_velocity_selection.asDiagonal();

    Matrix6d Jac0, Jac0_inv;
    Jac0_inv = JacobianSpt2BodyV(SE3_WT_fb.block<3, 3>(0, 0));
    Jac0 = Jac0_inv.inverse();

    MatrixXd m_anni = _m_velocity_selection*Tr*Jac0;
    Vector6d spt_TSo_new = (Matrix6d::Identity() -
            pseudoInverse(m_anni, 1e-6)*m_anni)*spt_TSo;
    // update offset
    _SE3_WToffset = SE3_WT_fb*spt2SE3(spt_TSo_new)*SE3Inv(SE3_WTSet);

    // project these into force space
    _wrench_T_Err_I = _Tr_inv*_m_force_selection*Tr*_wrench_T_Err_I;
    _wrench_T_Err   = _Tr_inv*_m_force_selection*Tr*_wrench_T_Err;

    _SE3_WT_old = SE3_WT_fb;
    _Tr         = Tr;
    _Tr_inv     = _Tr.inverse();

    if (isnan(_SE3_WToffset(0,0))) {
        cout << "SE3_WT_fb:\n" << SE3_WT_fb.format(MatlabFmt) << endl;
        cout << "SE3_TSo:\n" << SE3_TSo.format(MatlabFmt) << endl;
        cout << "spt_TSo:\n" << spt_TSo.format(MatlabFmt) << endl;
        cout << "Jac0_inv:\n" << Jac0_inv.format(MatlabFmt) << endl;
        cout << "Jac0:\n" << Jac0.format(MatlabFmt) << endl;
        cout << "m_anni:\n" << m_anni.format(MatlabFmt) << endl;
        cout << "spt_TSo_new:\n" << spt_TSo_new.format(MatlabFmt) << endl;
        cout << "_SE3_WToffset:\n" << _SE3_WToffset.format(MatlabFmt) << endl;
        cout << "\nNow paused at updateAxis()";
        getchar();
    }
}

// reset everytime you start from complete stop.
// clear up internal states
void ForceControlController::reset()
{
    _hw->getPose(_pose_sent_to_robot);
    _SE3_WT_old = posemm2SE3(_pose_sent_to_robot);
    _SE3_WToffset = Matrix4d::Identity();
    _v_W = Vector6d::Zero();
    _wrench_T_Err = Vector6d::Zero();
    _wrench_T_Err_I = Vector6d::Zero();
}

void ForceControlController::displayStates() {
    using namespace std;
    cout << "================= Parameters ================== " << endl;
    cout << "_ToolStiffnessMatrix: \n" << _ToolStiffnessMatrix.format(MatlabFmt)
            << endl;
    cout << "_ToolDamping_coef: \n" << _ToolDamping_coef.format(MatlabFmt)
            << endl;
    cout << "_ToolInertiaMatrix: \n" << _ToolInertiaMatrix.format(MatlabFmt)
            << endl;
    cout << "================= Commands ================== " << endl;
    cout << "_pose_user_input: ";
    stream_array_in(cout, _pose_user_input, 7);
    cout << "\n_Wrench_Tr_set: \n" << _wrench_Tr_set.format(MatlabFmt) << endl;
    cout << "_Tr: \n" << _Tr.format(MatlabFmt) << endl;
    cout << "_Tr_inv: \n" << _Tr_inv.format(MatlabFmt) << endl;
    cout << "_m_force_selection: \n" << _m_force_selection.format(MatlabFmt) << endl;
    cout << "_m_velocity_selection: \n" << _m_velocity_selection.format(MatlabFmt) << endl;
    cout << "================= Internal states ================== " << endl;
    cout << "_pose_sent_to_robot: ";
    stream_array_in(cout, _pose_sent_to_robot, 7);
    cout << "\n_SE3_WT_old: \n" << _SE3_WT_old.format(MatlabFmt) << endl;
    cout << "_SE3_WToffset: \n" << _SE3_WToffset.format(MatlabFmt) << endl;
    cout << "_v_W: \n" << _v_W.format(MatlabFmt) << endl;
    cout << "_wrench_T_Err: \n" << _wrench_T_Err.format(MatlabFmt) << endl;
    cout << "_wrench_T_Err_I: \n" << _wrench_T_Err_I.format(MatlabFmt) << endl;
}