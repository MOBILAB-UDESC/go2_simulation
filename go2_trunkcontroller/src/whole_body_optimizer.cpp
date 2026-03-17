/**
  * whole_body_optimizer.cpp
  *
  * Created on: 2025-03-04
  *     Author: stittel
  *   Based on: WholeBodyOptimizationWithArm from DLS1 (by mrisiglione)
  */

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <robotlib/utils/utils.hpp>
#include <dls2/math/rotations.hpp>

#include "controllers/trunk_controller/whole_body_optimizer.h"
#include "controllers/trunk_controller/eiquadprog.hpp"

//! number of base dof
static const int DOF_BASE = 6;
//! Number of arm joints
static const int JNTS_ARM = 0; // ToDo: only valid for 7-dof arms
//! Number of all joints including virtual base joints
static const int JNTS_ROB = 12; // ToDo: only valid for quadrupeds with 3-dof legs
//! Number of all joints including virtual base joints
static const int JNTS_ALL = JNTS_ROB + DOF_BASE;
//! Index at which robot joints start
static const int IDX_ROB = DOF_BASE;
//! Index at which ground reaction forces start
static const int IDX_GRF = JNTS_ALL;

static gboolean draw_cb(GtkWidget *widget, cairo_t *cr, gpointer data)
{
  /* Set color for background */
  cairo_set_source_rgb(cr, 1, 1, 1);
  /* fill in the background color*/
  cairo_paint(cr);

  int width = gtk_widget_get_allocated_width(widget);
  int height = gtk_widget_get_allocated_height(widget);
  int x0(width / 2), y0(height / 2);

  controllers::WholeBodyOptimizer* wbo = (controllers::WholeBodyOptimizer*) data;
  static Eigen::Vector3d com, trunkPos, desPos, feetCenter, lf, rf, lh, rh;
  wbo->getCom(com);
  wbo->getTrunkPos(trunkPos);
  wbo->getDesPos(desPos);
  wbo->getFeetCenter(feetCenter);
  wbo->getFeetPos(lf, rf, lh, rh);

  com -= trunkPos;
  desPos -= trunkPos;
  feetCenter -= trunkPos;
  lf -= trunkPos;
  rf -= trunkPos;
  lh -= trunkPos;
  rh -= trunkPos;
  trunkPos.setZero();

  static double scale(600);

  /* draw grid lines */
  cairo_set_source_rgb(cr, 0.77, 0.77, 0.77);
  cairo_set_line_width(cr, 0.4);
  for (double k = -1; k < 1; k += 0.1)
  {
    cairo_move_to(cr, x0 + scale * k, 0);
    cairo_line_to(cr, x0 + scale * k, height);
    cairo_move_to(cr, 0, y0 + scale * k);
    cairo_line_to(cr, width, y0 + scale * k);
  }
  cairo_stroke(cr);

  /* draw feet diagonals */
  cairo_set_source_rgb(cr, 0.77, 0.16, 0.13);
  cairo_set_line_width(cr, 1);
  cairo_move_to(cr, x0 + scale * lf(0), y0 - scale * lf(1));
  cairo_line_to(cr, x0 + scale * rh(0), y0 - scale * rh(1));
  cairo_move_to(cr, x0 + scale * rf(0), y0 - scale * rf(1));
  cairo_line_to(cr, x0 + scale * lh(0), y0 - scale * lh(1));
  cairo_stroke(cr);

  /* draw rectangle at desired pos */
  cairo_set_source_rgb(cr, 0.17, 0.63, 0.63);
  cairo_set_line_width(cr, 2);
  cairo_rectangle(cr, x0 + scale * desPos(0) - 4, y0 - scale * desPos(1) - 4, 8, 8);
  cairo_stroke(cr);

  /* draw + at feet center */
  cairo_set_source_rgb(cr, 0.17, 0.25, 0.80);
  cairo_set_line_width(cr, 2);
  cairo_move_to(cr, x0 + scale * feetCenter(0) - 4, y0 - scale * feetCenter(1));
  cairo_line_to(cr, x0 + scale * feetCenter(0) + 4, y0 - scale * feetCenter(1));
  cairo_move_to(cr, x0 + scale * feetCenter(0), y0 - scale * feetCenter(1) - 4);
  cairo_line_to(cr, x0 + scale * feetCenter(0), y0 - scale * feetCenter(1) + 4);
  cairo_stroke(cr);

  /* draw diamond at trunk position */
  cairo_set_source_rgb(cr, 0.17, 0.25, 0.80);
  cairo_set_line_width(cr, 2);
  cairo_move_to(cr, x0 + scale * trunkPos(0) - 4, y0 - scale * trunkPos(1));
  cairo_rel_line_to(cr, 4, 4);
  cairo_rel_line_to(cr, 4, -4);
  cairo_rel_line_to(cr, -4, -4);
  cairo_close_path(cr);
  cairo_stroke(cr);

  /* draw circle at com */
  cairo_set_source_rgb(cr, 0.17, 0.63, 0.12);
  cairo_set_line_width(cr, 2);
  cairo_arc(cr, x0 + scale * com(0), y0 - scale * com(1), 4, 0, 2 * G_PI);
  cairo_stroke(cr);

  /* draw legend */
  int legendX = width - 200;
  int legendY = 20;
  int lineHeight = 22;
  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, 13);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX, legendY);
  cairo_show_text(cr, "Legend");
  legendY += lineHeight;
  
  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size(cr, 11);
  
  // Grid
  cairo_set_source_rgb(cr, 0.77, 0.77, 0.77);
  cairo_set_line_width(cr, 3);
  cairo_move_to(cr, legendX, legendY - 5);
  cairo_line_to(cr, legendX + 15, legendY - 5);
  cairo_stroke(cr);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX + 20, legendY);
  cairo_show_text(cr, "Grid (0.1m)");
  legendY += lineHeight;
  // Feet diagonals
  cairo_set_source_rgb(cr, 0.77, 0.16, 0.13);
  cairo_set_line_width(cr, 2);
  cairo_move_to(cr, legendX, legendY - 5);
  cairo_line_to(cr, legendX + 15, legendY - 5);
  cairo_stroke(cr);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX + 20, legendY);
  cairo_show_text(cr, "Feet diagonals");
  legendY += lineHeight;
  // Desired position
  cairo_set_source_rgb(cr, 0.17, 0.63, 0.63);
  cairo_set_line_width(cr, 2);
  cairo_rectangle(cr, legendX + 4, legendY - 10, 8, 8);
  cairo_stroke(cr);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX + 20, legendY);
  cairo_show_text(cr, "Desired pos");
  legendY += lineHeight;
  // Feet center
  cairo_set_source_rgb(cr, 0.17, 0.25, 0.80);
  cairo_set_line_width(cr, 2);
  cairo_move_to(cr, legendX + 4, legendY - 5);
  cairo_line_to(cr, legendX + 12, legendY - 5);
  cairo_move_to(cr, legendX + 8, legendY - 9);
  cairo_line_to(cr, legendX + 8, legendY - 1);
  cairo_stroke(cr);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX + 20, legendY);
  cairo_show_text(cr, "Feet center");
  legendY += lineHeight;
  // Trunk position
  cairo_set_source_rgb(cr, 0.17, 0.25, 0.80);
  cairo_set_line_width(cr, 2);
  cairo_move_to(cr, legendX + 4, legendY - 5);
  cairo_rel_line_to(cr, 4, 4);
  cairo_rel_line_to(cr, 4, -4);
  cairo_rel_line_to(cr, -4, -4);
  cairo_close_path(cr);
  cairo_stroke(cr);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX + 20, legendY);
  cairo_show_text(cr, "Trunk pos");
  legendY += lineHeight;
  // CoM
  cairo_set_source_rgb(cr, 0.17, 0.63, 0.12);
  cairo_set_line_width(cr, 2);
  cairo_arc(cr, legendX + 8, legendY - 5, 4, 0, 2 * G_PI);
  cairo_stroke(cr);
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_move_to(cr, legendX + 20, legendY);
  cairo_show_text(cr, "CoM");

  /* draw height line on the left */
  cairo_set_source_rgb(cr, 0, 0, 0);
  cairo_set_line_width(cr, 1);
  cairo_move_to(cr, 20, 50);
  cairo_line_to(cr, 20, 350);
  cairo_stroke(cr);

  /* draw rectangle at desired height */
  cairo_set_source_rgb(cr, 0.17, 0.63, 0.63);
  cairo_set_line_width(cr, 2);
  cairo_rectangle(cr, 20 - 4, 350 - 300 * desPos(2) - 4, 8, 8);
  cairo_stroke(cr);

  /* draw diamond at trunk position */
  cairo_set_source_rgb(cr, 0.17, 0.25, 0.80);
  cairo_set_line_width(cr, 2);
  cairo_move_to(cr, 20 - 4, 350 - 300 * trunkPos(2));
  cairo_rel_line_to(cr, 4, 4);
  cairo_rel_line_to(cr, 4, -4);
  cairo_rel_line_to(cr, -4, -4);
  cairo_close_path(cr);
  cairo_stroke(cr);

  return 0;
}

void callGtkMain()
{
  gtk_main();
}

void markerTask(controllers::WholeBodyOptimizer* wbo)
{
  if (wbo)
  {
    gtk_init(0, NULL);

    GtkWidget* window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Feet diagonals and COM");
    g_signal_connect(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_window_set_keep_above(GTK_WINDOW(window), true);
    gtk_window_set_accept_focus(GTK_WINDOW(window), false);
    // gtk_window_move(GTK_WINDOW(window), 100, 20);

    GtkWidget* da = gtk_drawing_area_new();
    gtk_widget_set_size_request (da, 600, 400);
    g_signal_connect(da, "draw", G_CALLBACK(draw_cb), gpointer(wbo));

    gtk_container_add(GTK_CONTAINER (window), da);
    gtk_widget_show(da);
    gtk_widget_show(window);

    std::thread gtkThread(callGtkMain);

    while (wbo) wbo->updateMarkers(window);

    gtkThread.join();
  }
}

namespace controllers
{
  WholeBodyOptimizer::WholeBodyOptimizer(
      const std::shared_ptr<robotlib::RobotBase>& robot)
  : _robot_ptr(robot)
  , _joint_positions(robot->makeJointState())
  , _joint_velocities(_joint_positions)
  , _joint_accelerations(_joint_positions)
  , _joint_torques(_joint_positions)
  , _nle_joints(_joint_positions)
  , _inertia_js(robot->getNJOINTS(), robot->getNJOINTS())
  , _kp_ost_arm(700, 700, 700)
  , _kd_ost_arm(50, 50, 50)
  , _kp_oso_arm(700, 700, 700)
  , _kd_oso_arm(50, 50, 50)
  , _kp_posture_arm(500, 450, 500, 500, 500, 500, 600)
  , _kd_posture_arm(200, 200, 200, 200, 150, 150, 300)
  , _feet_forces(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
  , _surf_normal(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d(0, 0, 1)))
  , _mu_estimate(robot->makeLimbDataMap<double>(0.4))
  , _stance_legs(robot->makeLimbDataMap<bool>(true))
  , _force_max(0.0)
  , _force_min(0.0)
  , _des_swing_pos(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
  , _des_swing_vel(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
  , _des_swing_acc(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
  , _foot_pos_map(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
  , _foot_rot_map(robot->makeLimbDataMap<Eigen::Matrix3d>(Eigen::Matrix3d::Zero()))
  , _foot_vel_map(robot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
  , _foot_twist_map(robot->makeLimbDataMap<robotlib::Vec6d>(robotlib::Vec6d::Zero()))
  , _foot_acc_map(robot->makeLimbDataMap<robotlib::Vec6d>(robotlib::Vec6d::Zero()))
  , _foot_jac_map(robot->makeLimbDataMap<Eigen::Matrix3d>(Eigen::Matrix3d::Zero()))
  , _arm_pose(Eigen::Matrix4d::Identity())
  , _joints_arm_ref_pos(2.3562, -0.7854, 0.0, -0.7854, .0, .0, 1.5708)
  , _joints_arm_ref_vel(0, 0, 0, 0, 0, 0, 0)
  , _joints_arm_ref_acc(0, 0, 0, 0, 0, 0, 0)
  , _arm_jac(Eigen::MatrixXd::Identity(6, 6))
  , _base_rot_inv(Eigen::Matrix3d::Identity())
  , _wrench_des(0, 0, 0, 0, 0, 0)
  , _wrench_ee(0, 0, 0, 0, 0, 0)
  // ToDo: Those sizes are wrong. They are placeholders since the real sizes are computed dynamically
  , _wght_arm(50)
  , _wght_wrench(20)
  , _cleg_cnt(0)
  , _num_eq(0)
  , _num_ineq(0)
  , _num_slacks(0)
  , _fric_cnstr(0)
  , _x(38)
  , _GQ(38, 38)
  , _g0(38)
  , _CE(18, 38)
  , _ce0(18)
  , _CI(25, 38)
  , _ci0(25)
  , _cc_torques(robot->getNJOINTS())
  , _running(false)
  {

    _x.setZero();
    _GQ.setZero();
    _g0.setZero();
    _CE.setZero();
    _ce0.setZero();
    _CI.setZero();
    _ci0.setZero();
    _cc_torques.setZero();
    _base_wrench.setZero();

    _com.setZero();
    _des_pos.setZero();
    _marker_thread.reset(new std::thread(markerTask, this));
  }

  WholeBodyOptimizer::~WholeBodyOptimizer()
  {
  }

  void WholeBodyOptimizer::computeDummyMotionInSpace()
  {
    static double k(0), s, c, time(0),
                  flh, flt, flk,
                  frh, frt, frk,
                  blh, blt, blk,
                  brh, brt, brk;
    static int i;
    bool prnt(std::fmod(time, 1.0) < 0.001);
    i = 0;
    s = 2 * std::sin(time * 10);
    c = 2 * std::cos(time * 10);
    if (prnt) std::cout << "[WBC] " << time << ": " << s << "; " << c;
    static std::string separators[] = { "\n      fl: ", ", ", ", ",
                                        "\n      fr: ", ", ", ", ",
                                        "\n      bl: ", ", ", ", ",
                                        "\n      br: ", ", ", ", " };
    for (auto joint : _robot_ptr->getJoints())
    {
      if (prnt) std::cout << separators[i] << _joint_positions[joint->id];

      if (i % 3 == 0)
      {
        _joint_torques[joint->id] = 200 * (0.0 - _joint_positions[joint->id])
                              - 10 * (_joint_velocities[joint->id]);
      }
      else if (i % 3 == 1)
      {
        // k = (i == 1 || i == 10) ? -s : s;
        _joint_torques[joint->id] = 800 * (0.7 - _joint_positions[joint->id]) + k
                              - 100 * (_joint_velocities[joint->id]);
      }
      else
      {
        // k = (i == 2 || i == 11) ? -c : c;
        _joint_torques[joint->id] = 800 * (-1.5 - _joint_positions[joint->id]) + k
                              - 10 * (_joint_velocities[joint->id]);
      }
      i++;
    }
    if (prnt) std::cout << std::endl;
    time += 0.001;
  }

#define CART_DOF 6
#define CNTCT_CNSTRNTS 3
#define LEGS 4

  void WholeBodyOptimizer::computeOptimizationWithArm(
      const robotlib::LimbDataMap<Eigen::Vector3d>& surfNormal,
      const robotlib::LimbDataMap<double>& muEstimate,
      const robotlib::LimbDataMap<bool>& stanceLegs,
      const double& forceMax,
      const double& forceMin,
      const dls::utils::Pose& basePose,
      const dls::utils::Screw& baseTwist,
      const robotlib::JointState& q,
      const robotlib::JointState& qd,
      const robotlib::JointState& desQ,
      const robotlib::Vec6d& desWrench,
      const Eigen::Vector3d& comShift)
  {
    static std::ofstream logFile("tmp/wbo.txt");
    // logFile << "set variables" << std::endl;

    _surf_normal = surfNormal;
    _mu_estimate = muEstimate;
    _stance_legs = stanceLegs;
    _force_max = forceMax;
    _force_min = forceMin;
    _base_pose = basePose;

    _base_twist = baseTwist;
    _joint_positions = q;
    _joint_velocities = qd;
    _wrench_des = desWrench;

    _pos << _base_pose.toPosition();
    _rot << _base_pose.toRotationMatrix(); // from base to world

    /////////////////////////////////////////////////////////////
    // ToDo: why does body center without gains after walking? //
    /////////////////////////////////////////////////////////////

    _jnt_vel_eig = _joint_velocities;

    // find number of stance legs
    _cleg_cnt = 0;
    for (auto leg : _robot_ptr->getLegs()) if (_stance_legs[leg]) _cleg_cnt++;
    ////////////////useful variables//////////////////////
    _cntct_frcs = CNTCT_CNSTRNTS * _cleg_cnt;
    // equality constraints
    _swng_cnstr = CNTCT_CNSTRNTS * (LEGS - _cleg_cnt);
    // ineq constraints
    _fric_cnstr = CART_DOF * _cleg_cnt;

    // TODO: Understand if this line is needed
    //////////////////////////////////////////////////////////////////////////////////
    // push com above feet center if all legs are stance
    // if (_cleg_cnt == 4) _wrench_des.segment<2>(3) -= 500 * (_rot * feetToCom).head(2);
    //////////////////////////////////////////////////////////////////////////////////

    // compute by-products
    // logFile << "prepareOptimization" << std::endl;
    prepareOptimization();

    // logFile << "_num_eq" << std::endl;
    // compute the number of constraints (needs to be done after inizialized)
    _num_eq = _cntct_frcs + CART_DOF;
    _num_ineq = _fric_cnstr + _swng_cnstr * 2;
    _num_slacks = 1 + _swng_cnstr;

    // update useful transforms peculiar only of the dynamic case
    // logFile << "computeJSInertiaMatrix" << std::endl;
    // static const Eigen::Matrix<double, 7, 1> robotPose(0, 0, 0, 0, 0, 0, 1);
    // _robot_ptr->computeJSInertiaMatrix(robotPose,
    _robot_ptr->computeJSInertiaMatrix(_base_pose.toVector(),
                                       _joint_positions,
                                       _inertia_js); // Robotlib gives joint space inertia matrix

    // set cost function
    // logFile << "setCostFunction();" << std::endl;
    setCostFunction();
    // set equality constraints
    // logFile << "setEqualities();" << std::endl;
    setEqualities();
    // set ineuqality constraints
    // logFile << "setInequalities();" << std::endl;
    setInequalities(desQ);
    // logFile << "zero x" << std::endl;

    _x.setZero();
    _joint_torques.setZero();
    for (auto leg : _robot_ptr->getLegs()) _feet_forces[leg].setZero();

    // static int cnt(0);
    // if (cnt++ % 1000 == 0)
    // {
    //   logFile << "feet jacobians:";
    //   logFile << "\nLF:\n" << _foot_jac_map[_robot_ptr->getLimb("LF")];
    //   logFile << "\nRF:\n" << _foot_jac_map[_robot_ptr->getLimb("RF")];
    //   logFile << "\nLH:\n" << _foot_jac_map[_robot_ptr->getLimb("LH")];
    //   logFile << "\nRH:\n" << _foot_jac_map[_robot_ptr->getLimb("RH")] << std::endl;
    // }

    // logFile << "solve" << std::endl;

    double result = Eigen::solve_quadprog(_GQ, _g0,
                                          _CE.transpose(), _ce0,
                                          _CI.transpose(), _ci0,
                                          _x);
    // logFile << "solved" << std::endl;

    // check if a solution was found
    if (result == std::numeric_limits<double>::infinity())
    {
      std::cout << "Couldn't find a feasible solution" << std::endl;
    }
    else
    {
      // Actual torque computation: tau = M*qdd + h - Jc^T*lambda
      _joint_torques = _inertia_js.block(IDX_ROB, 0, JNTS_ROB, JNTS_ALL)
                     * _x.segment(0, JNTS_ALL)
                     + _cc_torques
                     - _jac_stnc.block(0, IDX_ROB, _cntct_frcs, JNTS_ROB).transpose()
                     * _x.segment(IDX_GRF, _cntct_frcs);
      // logFile << "get feet forces" << std::endl;

      // map feet forces into a joint state vector because the number of contact forces is variable a for loop is needed
      int cntctIndx = IDX_GRF;
      for (auto leg : _robot_ptr->getLegs()) if (_stance_legs[leg])
      {
        _feet_forces[leg] = _x.segment(cntctIndx, CNTCT_CNSTRNTS);
        cntctIndx += CNTCT_CNSTRNTS;
      }
    }
    // logFile << "done" << std::endl;

    _running = true;
  }

#define b33 block<3, 3> // extract 3x3 blocks from Eigen matrix (args: start row & col)
#define h3 head<3> // extract first 3 elements from Eigen vector
#define s3 segment<3> // extract 3 elements from Eigen vector (arg: start row)
#define t3 tail<3> // extract last 3 elements from Eigen vector
  void WholeBodyOptimizer::prepareOptimization()
  {
    _robot_ptr->forwardKinematics(_joint_positions, _foot_pos_map); // foot positions in base frame
    _base_rot = _base_pose.toRotationMatrix(); // from base to world
    _base_rot_inv = _base_rot.transpose(); // from world to base

    _jac_stnc.setZero(_cntct_frcs, JNTS_ALL);
    _jac_swng.setZero(_swng_cnstr, JNTS_ALL);

    static Eigen::Matrix<double, 7, 1> zeroPose(0, 0, 0, 0, 0, 0, 1);
    static Eigen::Matrix3d zeroRot(Eigen::Matrix3d::Identity());
    // switching columns of front right and rear left legs (pinocchio-urdf-mismatch)
    // static int cols[] = {6, 12, 9, 15}; // TODO: NO HARDCODE
    int c(0), rStnc(0), rSwng(0);
    for (auto leg : _robot_ptr->getLegs())
    {
      // // ToDo: Check if the following adjustment is necessary!
      // // adjust the contact foot positions taking into account the distance
      // // between the real contact point and the center of the (spherical) foot.
      // footPos[LegID(leg)] = fwd_kin.getFootPos(q.segment(0, 12), LegID(leg)) - 0.02 * surf_normal[dog::LegID(leg)];

      // store computed jacobians for reuse
      auto footlink = leg->getEndEffector();

      // The following is in base frame
      _robot_ptr->computeLimbsJacobian(_joint_positions, footlink->getName(), _foot_jac);  // Those are in base frame
      _foot_jac_map[leg] = _foot_jac.b33(0, 3*c);

      // // compute foot velocity in base frame via jacobian
      // _foot_vel_map[leg] = _foot_jac_map[leg] * _jnt_vel_eig.s3(3*c); // TODO: We don't use _foot_vel_map anywhere?

      _skew_mat << 0, -_foot_pos_map[leg](2), _foot_pos_map[leg](1),
                   _foot_pos_map[leg](2), 0, -_foot_pos_map[leg](0),
                   -_foot_pos_map[leg](1), _foot_pos_map[leg](0), 0;

      if (_stance_legs[leg])
      {
        _robot_ptr->computeWholeBodyJacobian(_joint_positions,
                                             footlink, _tmp_jac);
        _jac_stnc.b33(3*rStnc, 0) << -_base_rot_inv * _skew_mat;
        _jac_stnc.b33(3*rStnc, 3) << _base_rot_inv;
        _jac_stnc.b33(3*rStnc++, 6+3*c) << _base_rot_inv * _tmp_jac.b33(0, 6+3*c);
      }
      else
      {
        _robot_ptr->computeWholeBodyJacobian(_joint_positions,
                                             footlink, _tmp_jac);
        _jac_swng.b33(3 * rSwng, 0) << -zeroRot * _skew_mat;
        _jac_swng.b33(3 * rSwng, 3) << zeroRot;
        _jac_swng.b33(3 * rSwng++, 6 + 3 * c) << zeroRot * _tmp_jac.b33(0, 6 + 3 * c);
      }
      c++;
    }

    // old code as reference:
    // Jcq = Jc.block(0, activeJoints, _cntct_frcs, JNTS_ALL); // Jcq is in the world frame to map forces in the world into torques
    // Jcb = Jc.block(0, dog::baseJoints, _cntct_frcs, 6);		  // Jcb is in the world frame

    _robot_ptr->computeNonLinearEffects(_base_pose.toVector(), _base_twist.data(),
                                        _joint_positions, _joint_velocities,
                                        _nle_base, _nle_joints);
    _cc_torques << _nle_joints;

    // not used, but imported to remember the meaning of h (ToDo: remove)
    // h.segment(dog::baseJoints, 6) = baseWrench;
    // h.segment(activeJoints, JNTS_ALL) = h_joints;		// centrifugal and Coriolis torques of all (legs+arm) joints

    // the following has been moved here from setEqualities() and setInequalities()

    // 1-enforce contact constraints (9/12) Jc qdd + Jcdqd=0 zero accel in world frame
    // b_baseTwist is in base frame and computed JcdQd should be in world frame
    // compute stance foot jacobian multiplied by joint velocities
    _robot_ptr->forwardKinematics(_joint_positions, _joint_velocities,
                                  _joint_accelerations, _foot_pos_map,
                                  _foot_rot_map, _foot_twist_map, _foot_acc_map);
    _jac_stnc_qd.resize(_cntct_frcs);
    _jac_swng_qd.resize(_swng_cnstr);
    rStnc = 0;
    rSwng = 0;
    for (auto leg : _robot_ptr->getLegs())
    {
      // remove coriolis in linear part to get euclidean acceleration
      _foot_acc_map[leg].t3() += _foot_twist_map[leg].h3().cross(
                                 _foot_twist_map[leg].t3());
      if (_stance_legs[leg])
      {
        _jac_stnc_qd.s3(rStnc) = _base_rot_inv * _foot_rot_map[leg]
                               * _foot_acc_map[leg].t3();
        rStnc += 3;
      }
      else
      {
        _jac_swng_qd.s3(rSwng) = _foot_rot_map[leg] * _foot_acc_map[leg].t3();
        rSwng += 3;
      }
    }
  }

  void WholeBodyOptimizer::setCostFunction()
  {
    // initialize local matrix
    _b.setZero();
    _b_arm.setZero();
    _A.resize(6, JNTS_ALL + _cntct_frcs);
    _A.setZero();
    _A_arm.resize(JNTS_ARM, JNTS_ALL + _cntct_frcs);
    _A_arm.setZero();

    // resize x
    _x.resize(JNTS_ALL + _cntct_frcs + _num_slacks);
    _x.setZero();

    // resize input matrix
    _GQ.resize(JNTS_ALL + _cntct_frcs + _num_slacks, JNTS_ALL + _cntct_frcs + _num_slacks);
    _GQ.setZero();
    _g0.resize(JNTS_ALL + _cntct_frcs + _num_slacks);
    _g0.setZero();

    // Matrix to penalize decision variables
    _W.resize(JNTS_ALL + _cntct_frcs, JNTS_ALL + _cntct_frcs);
    _W.setIdentity();
    // _W *= 0.01;

    // consistency between joint acceleration and feet forces will enforced by equality constraints later
    // feet forces are already in world frame so they should not be rotated to be mapped into wrenches
    if (_A.rows() < 6 || _A.cols() < 6)
      std::cerr << "_A has wrong size" << std::endl;
    _A.b33(0, 0) = _inertia_js.b33(0, 0);
    _A.b33(3, 3) = _inertia_js.b33(3, 3);

    int index = JNTS_ALL;
    for (auto leg : _robot_ptr->getLegs())
    {
      if (_stance_legs[leg])
      {
        static Eigen::Vector3d wTorques(5, 1, 0.2);
        // Adding penalization of torque leg joints
        if (_W.rows() < index + 3 || _W.cols() < index + 3)
          std::cerr << "_W has wrong size" << std::endl;
        _W.b33(index, index) = _base_rot_inv * (_foot_jac_map[leg]) * 0.01
                             * wTorques.asDiagonal()
                             * _foot_jac_map[leg].transpose()
                             * _base_rot;
        index += CNTCT_CNSTRNTS;
      }
    }

    computeArmTrackingTaskJointSpace(); // ToDo: fill _b_arm in this method

    // set arm tracking matrix
    if (_A_arm.rows() < JNTS_ARM || _A_arm.cols() < JNTS_ALL)
      std::cerr << "_A_arm has wrong size" << std::endl;
    _A_arm.block(0, JNTS_ALL - JNTS_ARM, JNTS_ARM, JNTS_ARM).setIdentity();

    // Wrench already in base frame
    _b.s3(0) = _wrench_des.s3(0);
    _b.s3(3) = _wrench_des.s3(3);
    // NB: THIS IS THE COST!
    // The cost for the optimizer is assumed in the form x' * GQ * x + g0 x.
    // The goal is to find x that minimizes a certain cost function that we can call f.
    // In our case, f is expressed as
    // f = (A x - b)' S (A x - b) + (Aarm x - barm)' Sarm (Aarm x - barm) + x' W x.
    // The first term (...)' S (...) is tracking a desired wrench on the base.
    // The second term (...)' Sarm (...) tracks desired arm accelerations.
    // The third term is to minimize some decision variables,
    // in this case we are penalizing the ground reaction forces.
    // By expanding the f term, we get to:
    // f = (A x - b)' S (A x - b) + (Aarm x - barm)' Sarm (Aarm x - barm) + x' W x
    //   = x' A' S A x - 2 x' A' S b + b' S b + x' Aarm' Sarm Aarm x
    //   - 2x'Aarm' Sarm barm + barm' Sarm barm + x'Wx.
    // Now, since the cost is in the form x' * GQ * x + g0 for the optimizer,
    // we can group terms and end up to
    // GQ = A' S A + W + Aarm' Sarm Aarm
    // g0 = -2 x' A' S b - 2 x' Aarm' Sarm barm
    // *Note: Sarm = _wght_arm (all arm joints are weighted equally)
    //        S = _wght_wrench (all wrench dof are weighted equally)

    if (_GQ.rows() < JNTS_ALL + _cntct_frcs + _num_slacks ||
        _GQ.cols() < JNTS_ALL + _cntct_frcs + _num_slacks)
      std::cerr << "_GQ has wrong size" << std::endl;
    _GQ.block(0, 0, JNTS_ALL + _cntct_frcs, JNTS_ALL + _cntct_frcs) =
        _A.transpose() * _wght_wrench * _A + _W; //+ _A_arm.transpose() * _wght_arm * _A_arm;
    if (_g0.rows() < JNTS_ALL + _cntct_frcs)
      std::cerr << "_g0 has wrong size" << std::endl;
    _g0.segment(0, JNTS_ALL + _cntct_frcs) =
        -_b.transpose() * _wght_wrench * _A; // - _b_arm.transpose() * _wght_arm * _A_arm;

    // add slacks
    int slackIdx = JNTS_ALL + _cntct_frcs;
    for (int i = 0; i < _num_slacks; i++)
    {
      _GQ(slackIdx, slackIdx) = (i > 0) ? 100 : 1e6;
      slackIdx++;
    }
  }

  void WholeBodyOptimizer::setEqualities()
  {
    // init matrix
    _CE.resize(_num_eq, JNTS_ALL + _cntct_frcs + _num_slacks);
    _CE.setZero();
    _ce0.resize(_num_eq);
    _ce0.setZero();

    // _jac_stnc_qd calculation has been moved to prepareOptimization()

    // set the constraints
    _CE.block(0, 0, _cntct_frcs, JNTS_ALL) = _jac_stnc; // in world frame
    _ce0.segment(0, _cntct_frcs) = _jac_stnc_qd;

    // This method is used to obtain the arm contact Jacobian in the base frame.
    // The contact point is expressed at the bracelet point of the arm.
    // static auto trunk = _robot_ptr->getLink("TRUNK");
    // static auto bracelet = _robot_ptr->getLink("ARM_L7");
    // _arm_pose = _robot_ptr->computeFramePose(_joint_positions, bracelet, trunk);
    // static Eigen::MatrixXd wristToBrace(Eigen::MatrixXd::Identity(4, 4));
    // // ToDo: remove hard coded stuff (additional frame may be added to urdf)
    // //       -> after that, multiplication can be removed completely and
    // //          only arm position is needed, not the whole frame pose
    // wristToBrace.block<3, 1>(0, 3) << 0.000281, 0.011402, -0.029798;
    // _arm_pose *= wristToBrace;
    // _arm_jac.b33(0, 3) <<                0, -_arm_pose(2, 3),  _arm_pose(1, 3),
    //                        _arm_pose(2, 3),                0, -_arm_pose(0, 3),
    //                       -_arm_pose(1, 3),  _arm_pose(0, 3),                0;

    // 2-enforce consistency beween joint accel constraints and feet forces (6), PCC!
    // to avoid complications since the output wrench of Cterms and Gterms are in base
    // frame we write the constraint in base frame, thus evaluating Jcb in base frame
    _CE.block(_cntct_frcs, 0, CART_DOF, JNTS_ALL)
        = _inertia_js.block(0, 0, CART_DOF, JNTS_ALL);
    _CE.block(_cntct_frcs, JNTS_ALL, CART_DOF, _cntct_frcs)
        = -_jac_stnc.block(0, 0, _cntct_frcs, CART_DOF).transpose();

    // h.segment(dog::baseJoints, 6) = baseWrench;
    // h.segment(activeJoints, JNTS_ALL) = h_joints;		// centrifugal and Coriolis torques of all (legs+arm) joints
    // h has been computed in prepare optimization
    _ce0.segment(_cntct_frcs, CART_DOF) << _nle_base.t3(), _nle_base.h3();
    // ToDo: get ee wrench as input
    robotlib::Vec6d eeWrenchEe(robotlib::Vec6d::Zero());
    // including in the pcc also the forces acting on the arm
    //_ce0.segment(_cntct_frcs, CART_DOF) -= _arm_jac * eeWrenchEe; // TODO: Readd when the arm is used or after refactoring
  }

  void WholeBodyOptimizer::setInequalities(const robotlib::JointState& desQ)
  {
    // init matrix
    _CI.setZero(_num_ineq + _num_slacks, JNTS_ALL + _cntct_frcs + _num_slacks);
    _ci0.setZero(_num_ineq + _num_slacks);

    // set friction cone limits
    Eigen::Vector3d tangentDir1, tangentDir2;
    Eigen::MatrixXd cone(4, 3); // ToDo: might be replaced by map to _CI block,
                                //       but be careful with mapping blocks!
    int stncIdx = 0; // stance leg index counter
    for (auto leg : _robot_ptr->getLegs())
    {
      if (_stance_legs[leg])
      {
        // 0 constraint -- fi.n >= 0  no pulling forces, only pushing...
        // 1 constraint -- fi.n <= Fzmax => -fi.n >= -Fzmax limit normal force
        _CI.block<2, 3>(CART_DOF * stncIdx, JNTS_ALL + 3 * stncIdx)
            << _surf_normal[leg].transpose(), -_surf_normal[leg].transpose();
        _ci0.segment<2>(CART_DOF * stncIdx) << -_force_min, _force_max;

        // add 2-6 cone constraints
        // for each tangent direction t,
        // we want: -mu*n . fi <= fi . t <= mu*n . fi,
        //(n.fi == normal component of force, t.fi = tangential component along vector t)
        // which is equivalent to the two constraints: mu*n . fi >= - fi . t and mu*n . fi >= fi . t, or equivalently
        // mu*n . fi + fi . t >=0 and mu*n . fi - fi . t >= 0

        // compute tanget components
        tangentDir1 = Eigen::Vector3d::UnitX().cross(_surf_normal[leg]).normalized();
        tangentDir2 = _surf_normal[leg].cross(tangentDir1).normalized();
        // compute tanget components (old):
        //   tangentDir1 = _surf_normal[leg].cross(Vector3d::UnitY());
        //   tangentDir1.normalize();
        //   tangentDir2 = _surf_normal[leg].cross(tangentDir1);
        //   tangentDir2.normalize();
        // compute 4 constraints
        cone << (_surf_normal[leg] * _mu_estimate[leg] + tangentDir1).transpose(),
                (_surf_normal[leg] * _mu_estimate[leg] - tangentDir1).transpose(),
                (_surf_normal[leg] * _mu_estimate[leg] + tangentDir2).transpose(),
                (_surf_normal[leg] * _mu_estimate[leg] - tangentDir2).transpose();

        _CI.block<4, 3>(CART_DOF * stncIdx + 2, JNTS_ALL + 3 * stncIdx) << cone;
        _ci0.segment<4>(CART_DOF * stncIdx + 2).setZero();

        stncIdx++;
      }
    }

    if (_num_slacks > 0)
    {
      // add slacks (I set only the slacks of ones (s1),
      // the other column (s2) if exist will be zeros and was already initialized
      _CI.block(0, JNTS_ALL + _cntct_frcs, _fric_cnstr, 1).setOnes();
    }

    // enforce swing constraints (if any) Jsw qdd  + (Jswdqd - Xsw_dd ) = 0
    // (in base Frame not in the world frame like before!)
    computeOperationalSpaceSwingtask(desQ);

    // these are the swing constraints
    // set equality (get only accel)
    _CI.block(_fric_cnstr, 0, _swng_cnstr, JNTS_ALL - JNTS_ARM) = _A_swing;
    _ci0.segment(_fric_cnstr, _swng_cnstr) = _b_swing;

    _CI.block(_fric_cnstr + _swng_cnstr, 0, _swng_cnstr, JNTS_ALL - JNTS_ARM) = -_A_swing;
    _ci0.segment(_fric_cnstr + _swng_cnstr, _swng_cnstr) = -_b_swing;

    if (_num_slacks > 0)
    {
      // ensure positivitiy of slacks
      // lower bound part slack of swing constraint CIx+ci0  > -eps, CIx+ci0+eps > 0
      // upper bound part slack CIx+ci0 < eps, CIx+ci0-eps < 0, -CIx-ci0+eps > 0
      int rStart(_fric_cnstr), cEnd(JNTS_ALL + _cntct_frcs + 1 + _swng_cnstr);
      for (int size : {_swng_cnstr, _swng_cnstr, _swng_cnstr + 1})
      {
        _CI.block(rStart, cEnd - size, size, size).setIdentity();
        rStart += size;
      }
    }
  }

  // this tasks are implemented at the accel level
  void WholeBodyOptimizer::computeOperationalSpaceSwingtask(const robotlib::JointState& desQ)
  {
    // swing jacobian calculation has been moved to prepareOptimization()

    // can have more than one swing leg at time!
    _A_swing.setZero(_swng_cnstr, JNTS_ALL - JNTS_ARM);
    _b_swing.setZero(_swng_cnstr);

    // Convert the joint position in desQ to _des_swing_pos (everything already in base frame)
    _robot_ptr->forwardKinematics(desQ, _des_swing_pos);

    Eigen::VectorXd swngAccDes;
    Eigen::Vector3d velErr, posErr;
    // Eigen::Vector3d com = _robot_ptr->computeWholeBodyCoM(_joint_positions); // TODO: remove if not needed
    swngAccDes.setZero(_swng_cnstr);
    int idx(0), sIdx(0);
    for (auto leg : _robot_ptr->getLegs())
    {
      if (!_stance_legs[leg])
      {
        posErr = _des_swing_pos[leg] - _foot_pos_map[leg];
        velErr = _des_swing_vel[leg] - _foot_jac_map[leg] * _jnt_vel_eig.s3(idx);
        swngAccDes.s3(sIdx) = _des_swing_acc[leg] + 2000 * posErr + 50 * velErr; // TODO: Remove _des_swing_acc[leg] is not in paper. Gains must be configurable!!
        sIdx += 3;
      }
      idx += 3;
    }

    _A_swing = _jac_swng;
    _b_swing = -swngAccDes + _jac_swng_qd;
  }

#undef s3
#undef t3
#undef h3
#undef b33

  // this method is used to compute joint accelerations to be tracked for the arm considering the reference motion in operational space
  void WholeBodyOptimizer::computeArmTrackingTaskJointSpace()
  {
    // resizing and setting variables
    _b_arm.setZero(JNTS_ARM);

    // ToDo: transfer code from old wbc when arm joint variables are available.
  }

  //////////////////////
  // traits for getters
  //////////////////////

  robotlib::LimbDataMap<Eigen::Vector3d>& WholeBodyOptimizer::getFeetForces()
  {
    return _feet_forces;
  }

  robotlib::JointState& WholeBodyOptimizer::getJointTorques()
  {
    return _joint_torques;
  }

  void WholeBodyOptimizer::updateJointState(
      const robotlib::JointState& positions,
      const robotlib::JointState& velocities,
      const robotlib::JointState& accelerations)
  {
    _joint_positions = positions;
    _joint_velocities = velocities;
    _joint_accelerations = accelerations;
  }

  std::vector<std::pair<std::string, int>> WholeBodyOptimizer::getDbgPairs()
  {
    static std::vector<std::pair<std::string, int>> _dbg_pairs
    {
      // std::pair<std::string, int>("trnk.trqs", 12),
      // std::pair<std::string, int>("wbo.matrices", 10),
      // // std::pair<std::string, int>("wbo._GQ", JNTS_ALL + 13),
      // // std::pair<std::string, int>("wbo._g0", JNTS_ALL + 13),
      // // std::pair<std::string, int>("wbo._CE", 38),
      // // std::pair<std::string, int>("wbo._ce0", 18),
      // // std::pair<std::string, int>("wbo._CI", 38),
      // // std::pair<std::string, int>("wbo._ci0", 37),
      // // std::pair<std::string, int>("wbo._x", JNTS_ALL + 13),
      // std::pair<std::string, int>("wbo.surf_normal", 12),
      // std::pair<std::string, int>("wbo.mu_estimate", 4),
      // std::pair<std::string, int>("wbo.stance_legs", 4),
      // std::pair<std::string, int>("wbo.force_max", 4),
      // std::pair<std::string, int>("wbo.force_min", 4),
      // std::pair<std::string, int>("wbo.base_pose", 7),
      // std::pair<std::string, int>("wbo.jnt_pos", 12),
      // std::pair<std::string, int>("wbo.jnt_vel", 12),
      // std::pair<std::string, int>("wbo.wrench_des", 6),
      // // std::pair<std::string, int>("wbo.swingFootRef", 12),
      // std::pair<std::string, int>("wbo.feet_forces", 12),
      // std::pair<std::string, int>("wbo._cc_torques", 12),
      // std::pair<std::string, int>("wbo.trqs", 12)
    };

    return _dbg_pairs;
  }

  void WholeBodyOptimizer::fillDbgVec(
      std::string& name,
      std::vector<double>& vec)
  {
    static const std::vector<robotlib::LimbPtr> legs = _robot_ptr->getLegs();

    if (name == "wbo.matrices")
    {
      vec[0] = _GQ.rows();
      vec[1] = _GQ.cols();
      vec[2] = _g0.rows();
      vec[3] = _CE.rows();
      vec[4] = _CE.cols();
      vec[5] = _ce0.rows();
      vec[6] = _CI.rows();
      vec[7] = _CI.cols();
      vec[8] = _ci0.rows();
      vec[9] = _x.rows();
    }
    else if (name == "wbo._GQ")
      for (int i = 0; i < _GQ.rows(); i++) vec[i] = _GQ(i, i);
    else if (name == "wbo._g0")
      for (int i = 0; i < _g0.rows(); i++) vec[i] = _g0(i);
    else if (name == "wbo._CE")
    for (int i = 0; i < _CE.rows(); i++) vec[i] = _CE(i, 0);
    else if (name == "wbo._ce0")
      for (int i = 0; i < _ce0.rows(); i++) vec[i] = _ce0(i);
    else if (name == "wbo._CI")
    for (int i = 0; i < _CI.rows(); i++) vec[i] = _CI(i, 0);
    else if (name == "wbo._ci0")
      for (int i = 0; i < _ci0.rows(); i++) vec[i] = _ci0(i);
    else if (name == "wbo._x")
      for (int i = 0; i < _x.rows(); i++) vec[i] = _x(i);
    else if (name == "wbo.surf_normal")
      for (int i = 0; i < vec.size(); i++) vec[i] = _surf_normal[legs[i/3]](i%3);
    else if (name == "wbo.mu_estimate")
      for (int i = 0; i < vec.size(); i++) vec[i] = _mu_estimate[legs[i]];
    else if (name == "wbo.stance_legs")
      for (int i = 0; i < vec.size(); i++) vec[i] = _stance_legs[legs[i]];
    else if (name == "wbo.force_max")
      for (int i = 0; i < vec.size(); i++) vec[i] = _force_max;
    else if (name == "wbo.force_min")
      for (int i = 0; i < vec.size(); i++) vec[i] = _force_min;
    else if (name == "wbo.base_pose")
    {
      static Eigen::Matrix<double, 7, 1> poseVec;
      poseVec << _base_pose.toVector();
      for (int i = 0; i < vec.size(); i++) vec[i] = poseVec(i);
    }
    else if (name == "wbo.jnt_pos")
      for (int i = 0; i < vec.size(); i++) vec[i] = _joint_positions(i);
    else if (name == "wbo.jnt_vel")
      for (int i = 0; i < vec.size(); i++) vec[i] = _joint_velocities(i);
    else if (name == "wbo.wrench_des")
      for (int i = 0; i < vec.size(); i++) vec[i] = _wrench_des(i);
    else if (name == "wbo.swingFootRef")
      for (int i = 0; i < vec.size(); i++) vec[i] = _des_swing_pos[legs[i/3]](i%3);
    else if (name == "wbo.feet_forces")
      for (int i = 0; i < vec.size(); i++) vec[i] = _feet_forces[legs[i/3]](i%3);
    else if (name == "wbo._cc_torques")
      for (int i = 0; i < vec.size(); i++) vec[i] = _cc_torques(i);
    else if (name == "wbo.trqs")
      for (int i = 0; i < vec.size(); i++) vec[i] = _joint_torques(i);
    else
      std::cerr << "[TRNK] dbg vec with name '" << name
                << "' does not exist in wbo!" << std::endl;
  }

  void WholeBodyOptimizer::updateMarkers(GtkWidget* window)
  {
    while (!_running) sleep(1);
    gtk_widget_queue_draw(window);
    usleep(100000); // 10 Hz
  }

  void WholeBodyOptimizer::getCom(Eigen::Vector3d& com)
  {
    com << _com;
  }

  void WholeBodyOptimizer::getDesPos(Eigen::Vector3d& desPos)
  {
    desPos << _des_pos;
  }

  void WholeBodyOptimizer::getTrunkPos(Eigen::Vector3d& trunkPos)
  {
    trunkPos << _pos;
  }

  void WholeBodyOptimizer::getFeetCenter(Eigen::Vector3d& feetCenter)
  {
    feetCenter << _feet_center;
  }

  void WholeBodyOptimizer::getFeetPos(Eigen::Vector3d& lf, Eigen::Vector3d& rf,
                                      Eigen::Vector3d& lh, Eigen::Vector3d& rh)
  {
    lf = _pos + _rot.transpose() * _foot_pos_map[_robot_ptr->getLimb("LF")];
    rf = _pos + _rot.transpose() * _foot_pos_map[_robot_ptr->getLimb("RF")];
    lh = _pos + _rot.transpose() * _foot_pos_map[_robot_ptr->getLimb("LH")];
    rh = _pos + _rot.transpose() * _foot_pos_map[_robot_ptr->getLimb("RH")];
  }

} // close namespace controllers
