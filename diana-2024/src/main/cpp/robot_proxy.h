#pragma once
#include <frc/TimedRobot.h>

struct RobotProxy : public frc::TimedRobot
{
	void RobotInit() override;
	void RobotPeriodic() override;
	
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	
	void TeleopInit() override;
	void TeleopPeriodic() override;
	
	void DisabledInit() override;
	void DisabledPeriodic() override;
	
	void TestInit() override;
	void TestPeriodic() override;
};
