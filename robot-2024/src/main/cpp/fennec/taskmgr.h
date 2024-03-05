#pragma once
#include <stdint.h>
#include <stddef.h>

#include "maths.h"
#include "pid.h"

struct RobotData; // forward declaration


#define TASKMGR_MAX_TASKS (1024)

enum TaskType {
	TASK_NONE = 0,

	TASK_LIST,

	TASK_DELAY,

	TASK_FOLLOW_LINE,

	TASK_WAYPOINT,
	TASK_DRIVETRAIN_VELOCITY,
	
	TASK_HANK,
	TASK_STAG,

	TASK_CHARGEPAD_STABLISE,

	TASK_UPDATE_MATE_STATE,

	TASK_CONE_ROTATION_ADJUST,
	TASK_CONE_Y_ADJUST,

	TASK_INTAKE_POSITIONING,
	TASK_INTAKE_PULLER,

	TASK_UPDATE_LIMELIGHT_LED_STATE,

    TASK_MIDDLE_THE_WHEELS,

	TASK_INTAKE,

	// TASK_FUNCTIONPTR, // TODO
};


struct TaskMgr;

struct TaskData_Delay {
	float length;

	//state
	float timer;
};

struct TaskData_Waypoint {
	Pose  target_pose;

	float speed;
	float speed_rot;
	float epsilon;
    float epsilon_rot;
};

struct TaskData_FollowLine {
	Pose starting_pose;
	Pose ending_pose;

	float start_speed;
	float mid_speed;
	float end_speed;

	float speed_rot;

	float max_accel;

	float epsilon;
    float epsilon_rot;
};

struct TaskData_DrivetrainVelocity {
	v2 target_velocity;
	float target_angular_velocity;

    float timer;
    float length;
};

struct TaskData_Hank {
	v2 target_state;
	float target_wrist;
	float epsilon;
	float epsilon_wrist;
	float timeout_time;
	float timeout_length;
};

struct TaskData_Stag {
	float intake_speed;
    float manual_offset;
    bool overrider;
};

struct TaskData_Shrek {
	float epsilon;
};

struct TaskData_Intake {
	float target_angle;
	float epsilon;
};

struct TaskData_IntakePuller {
	float throttle;
};

struct TaskData_MiddleWheels {
    bool enabled;
};


struct Task {
	TaskType type = TASK_NONE;
	bool started  = false;

	union {
		TaskMgr* list;
		TaskData_Delay delay;
		TaskData_Waypoint waypoint;
		TaskData_DrivetrainVelocity drivetrain_velocity;
		TaskData_Hank hank;
		TaskData_Stag stag;
		TaskData_Shrek shrek;
		TaskData_Intake intake;
		TaskData_IntakePuller intake_puller;
        TaskData_MiddleWheels middle_wheels;
        TaskData_FollowLine follow_line;

	};
};



struct TaskMgr
{
	uint64_t write_head = 0;
	uint64_t read_head = 0;

	// this is a ring buffer, keep that in mind
	Task task_buffer[TASKMGR_MAX_TASKS] = {};

	bool is_parallel = false;
};

bool pushTask(TaskMgr* mgr, Task task);
// void updateManager(TaskMgr* mgr, RobotData* robot);

// util
inline Task genTaskDefault(TaskType type)
{
	Task t;
	t.type = type;
	return t;
}

Task genTaskList(TaskMgr* list);

inline Task genTaskDelay(float delay)
{
	Task t;
	t.type = TASK_DELAY;
	t.delay.timer = 0;
	t.delay.length = delay;
	return t;
}

inline Task genTaskWaypoint(TaskData_Waypoint waypoint)
{
	Task t;
	t.type = TASK_WAYPOINT;
	t.waypoint = waypoint;
	return t;
}

inline Task genTaskDrivetrainVelocity(TaskData_DrivetrainVelocity drivetrain_velocity)
{
	Task t;
	t.type = TASK_DRIVETRAIN_VELOCITY;
	t.drivetrain_velocity = drivetrain_velocity;
	return t;
}

inline Task genTaskHank(TaskData_Hank hank)
{
	Task t;
	t.type = TASK_HANK;
	t.hank = hank;
	return t;
}

inline Task genTaskStag(TaskData_Stag stag)
{
	Task t;
	t.type = TASK_STAG;
	t.stag = stag;
	return t;
}

