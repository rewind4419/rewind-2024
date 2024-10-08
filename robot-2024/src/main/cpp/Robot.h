#pragma once

#include <thread>
#include <iostream>
// #include <photon/PhotonUtils.h>
#include <photon/PhotonCamera.h>
#include <frc/DriverStation.h>





// NavX2 library for the IMU
#include "AHRS.h"
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkFlex.h>


// FENNEC //
#include "fennec/drivetrain.h"
#include "fennec/drivetrain_controller.h"
#include "fennec/gamepad.h"
#include "fennec/pid.h"
#include "fennec/localiser.h"
#include "fennec/taskmgr.h"
#include "fennec/config.h"

////////////

#include "Intake.h"
#include "Shooter.h"
#include "RobotState.h"
#include "photonvis.h"
#include "projectilecalc.h"






enum RobotMode
{
  ROBOT_AUTO,
  ROBOT_TELEOP,
  ROBOT_DISABLE,
};


// Robot

enum Aligner
{
  ALGN_NONE=0,
  ALGN_FORWARD,
  ALGN_BACKWARD,
  ALGN_LEFT,
  ALGN_RIGHT,
};

struct RobotData
{
  bool manual_shooting_mode = false;
  int pose_calib_index = 0;
  // high level
  TaskMgr taskmgr;
  RobotStateContainer robotState;

  bool auto_first = true;
  float auto_init_delay = 0;

  std::optional<frc::DriverStation::Alliance> driverstation_side;
  float side = 0;
  // control
	float delta_time;
  float enable_time;
  float imu_basis;

  float auto_imu_basis;

  // RobotState lastCalledState;

  bool middle_wheels = false;
  Aligner aligner = ALGN_NONE;

  PID aligner_pid { .kP = 1.5, .kI = 0, .kD = 0.2 };
  PID holder_pid { .kP = 1.5, .kI = 0, .kD = 0.2 };

  Intake intake;
  Shooter shooter;

  PhotonParameters photon;

  ProjectileCalculations proj_calcs;

  Input input;
  Localiser_FirstOrderLag localiser;
  OdometryFrame latest_odometry_frame; 

  DrivetrainController drivetrain_controller;

  // physical components
  Drivetrain drivetrain;
  AHRS* sensor_imu;

  float driver_speed;

  float held_rotation;

  v2 global_input_translation;

  v3 integrated_imu_pos;

  // misc
  frc::Field2d field;

  bool ready_fire_amp = false;
}; 

void robotModeInit(RobotData *robot, RobotMode new_mode);
// void fieldDashboard(RobotData* r, v2 object, std::string object_name);

void initRobot(RobotData *r, RobotMode mode);
void updateRobot(RobotData *r, float time_step, RobotMode mode);


// Charge pad

// bool chargingPadStabilitazation(RobotData* r);

#if EXP_PATHING_MATTEO
Corners chargePad_evalPath(RobotData* r, v2 target_pose);
#endif

// Util

bool util_driveTo(RobotData* r, Pose target_pose, float speed, float speed_rot, float epsilon, float epsilon_rot);
