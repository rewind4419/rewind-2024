#include "RobotProxy.h" 
#include "Robot.h" 


/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

RobotData r;

void RobotProxy::RobotInit() {
        initRobot(&r, ROBOT_DISABLE);
}

void RobotProxy::RobotPeriodic() {
    
}

void RobotProxy::AutonomousInit() {
    robotModeInit(&r, ROBOT_AUTO);
}
void RobotProxy::AutonomousPeriodic() {
    // NOTE: robot timestep is hard-coded to 0.02 for now,
    // this is the default
    // if we ever change it, we should make a constant in the
    // Config.h file
    updateRobot(&r, 0.02, ROBOT_AUTO);
}

void RobotProxy::TeleopInit() {
    robotModeInit(&r, ROBOT_TELEOP);
}
void RobotProxy::TeleopPeriodic() {
    // NOTE: robot timestep is hard-coded to 0.02 for now,
    // this is the default
    // if we ever change it, we should make a constant in the
    // Config.h file
    updateRobot(&r, 0.02, ROBOT_TELEOP);
}


//////////////////////
// UNUSED

void RobotProxy::DisabledInit() 
{
    robotModeInit(&r, ROBOT_DISABLE);

}
void RobotProxy::DisabledPeriodic() {
    updateRobot(&r, 0.02, ROBOT_DISABLE);

}


void RobotProxy::TestInit() {

}
void RobotProxy::TestPeriodic() {
    printCalibrationData(&r.drivetrain);
}

void RobotProxy::SimulationInit() {}
void RobotProxy::SimulationPeriodic() {}

int main() {
  return frc::StartRobot<RobotProxy>();
}