#include "taskmgr.h"
#include "../Robot.h"

#include <stdio.h>


static bool taskStep(Task* task, RobotData* robot);

static void taskStart(Task* task, RobotData* robot)
{
	switch (task->type) {
		case TASK_WAYPOINT: {

		} break;

		default: break;
	}
}


static void doTask(TaskMgr* mgr, Task* task, RobotData* robot) {
	// Do the task
	if (!task->started) {
		task->started = true;
		taskStart(task, robot);
	}

	bool complete = taskStep(task, robot);
	if (complete) {
		if (task->type == TASK_LIST) free(task->list);
		*task = { };
		mgr->read_head = (mgr->read_head + 1) % TASKMGR_MAX_TASKS;
	}
}

void updateManager(TaskMgr* mgr, RobotData* robot) {
	if (mgr->is_parallel) {
		for (uint64_t i=0; i<mgr->write_head; i++) {
			doTask(mgr, &mgr->task_buffer[i], robot);
		}
	}
	else {
		if (mgr->read_head != mgr->write_head)
			doTask(mgr, &mgr->task_buffer[mgr->read_head], robot);
	}
}

static bool taskStep(Task* task, RobotData* robot)
{
	switch (task->type) {
	case TASK_LIST: {
		updateManager(task->list, robot);
		return task->list->read_head == task->list->write_head;
	} break;

	case TASK_DELAY: {
		task->delay.timer += robot->delta_time;
		return task->delay.timer > task->delay.length;
	} break;

	case TASK_WAYPOINT: {
		robot->drivetrain_controller.mode = DRIVECTRL_WAYPOINT;
		robot->drivetrain_controller.ctrl.waypoint.pose = task->waypoint.target_pose;
		robot->drivetrain_controller.ctrl.waypoint.speed = task->waypoint.speed;
		robot->drivetrain_controller.ctrl.waypoint.speed_rot = task->waypoint.speed_rot;

		if (length(robot->localiser.pose_estimate.position - task->waypoint.target_pose.position) < task->waypoint.epsilon
			&& fabsf(robot->localiser.pose_estimate.rotation - task->waypoint.target_pose.rotation) < task->waypoint.epsilon_rot)
		{
			return true;
		}

		return false;

	} break;

	case TASK_DRIVETRAIN_VELOCITY: {
		robot->drivetrain_controller.mode = DRIVECTRL_VELOCITY;
		robot->drivetrain_controller.ctrl.velocity.velocity = task->drivetrain_velocity.target_velocity;
		robot->drivetrain_controller.ctrl.velocity.angular_velocity = task->drivetrain_velocity.target_angular_velocity;

        task->drivetrain_velocity.timer += robot->delta_time;
		
		return task->drivetrain_velocity.timer > task->drivetrain_velocity.length;
	} break;

    case TASK_MIDDLE_THE_WHEELS: {
        robot->middle_wheels = task->middle_wheels.enabled;
        return true;
    }

	case TASK_FOLLOW_LINE: {

		TaskData_FollowLine* follow_line = &task->follow_line;

		v2 line_direction = normalize(follow_line->ending_pose.position - follow_line->starting_pose.position);
		float line_length = length(follow_line->ending_pose.position - follow_line->starting_pose.position);

		v2 our_pos_relative_to_start = robot->localiser.pose_estimate.position - follow_line->starting_pose.position;

		float dist_along_line = dot(line_direction, our_pos_relative_to_start);


		// t = (v1 - v0) / a
		// d = v0 * t + a * t ^ 2 / 2

		float mid_start_ideal_time = (follow_line->mid_speed - follow_line->start_speed) / follow_line->max_accel;
		float mid_start_length = follow_line->start_speed * mid_start_ideal_time + follow_line->max_accel * mid_start_ideal_time * mid_start_ideal_time / 2;

		float mid_end_ideal_time = (follow_line->mid_speed - follow_line->end_speed) / follow_line->max_accel; 
		float mid_end_length = follow_line->end_speed * mid_end_ideal_time + follow_line->max_accel * mid_end_ideal_time * mid_end_ideal_time / 2;


		float est_speed = follow_line->mid_speed;

		if (dist_along_line < mid_start_length)
		{
			est_speed = mix(follow_line->start_speed, follow_line->mid_speed, dist_along_line / mid_start_length);
		}
		else if (dist_along_line > line_length - mid_end_length)
		{
			est_speed = mix(follow_line->end_speed, follow_line->mid_speed, (line_length - dist_along_line) / mid_end_length);
		}

		if (dist_along_line < 0)
		{
			est_speed = follow_line->start_speed;
		}

		// printf("DIST: %f / %f, EST: %f\n", dist_along_line, line_length, est_speed);

		float tangence_to_line = dot(our_pos_relative_to_start, rightPerpendicular(line_direction));

		{

			v2 current_facing = rotate(v2{ 0, 1 }, robot->localiser.pose_estimate.rotation);
			v2 target_facing  = rotate(v2{ 0, 1 }, follow_line->ending_pose.rotation);

			float error = acos(dot(current_facing, target_facing));
			if (dot(target_facing, rightPerpendicular(current_facing)) < 0)
				error *= -1;

			float rot = error / M_PI;

			rot = evalPid(&robot->drivetrain_controller.aligner_pid, rot * robot->drivetrain_controller.ctrl.waypoint.speed_rot, robot->delta_time);


			v2 move = line_direction * est_speed - rightPerpendicular(line_direction) * tangence_to_line * 6;

			robot->drivetrain_controller.mode = DRIVECTRL_VELOCITY;
			robot->drivetrain_controller.ctrl.velocity.velocity = move;
			robot->drivetrain_controller.ctrl.velocity.angular_velocity = rot;
		}


		if (dist_along_line > line_length)
		{
			robot->drivetrain_controller.mode = DRIVECTRL_WAYPOINT;
			robot->drivetrain_controller.ctrl.waypoint.pose = follow_line->ending_pose;
			robot->drivetrain_controller.ctrl.waypoint.speed = follow_line->end_speed;
			robot->drivetrain_controller.ctrl.waypoint.speed_rot = follow_line->speed_rot;
		}


		if (length(robot->localiser.pose_estimate.position - follow_line->ending_pose.position) < follow_line->epsilon
			&& fabsf(robot->localiser.pose_estimate.rotation - follow_line->ending_pose.rotation) < follow_line->epsilon_rot)
		{
			robot->drivetrain_controller.mode = DRIVECTRL_WAYPOINT;
			robot->drivetrain_controller.ctrl.waypoint.pose = follow_line->ending_pose;
			robot->drivetrain_controller.ctrl.waypoint.speed = follow_line->end_speed;
			robot->drivetrain_controller.ctrl.waypoint.speed_rot = follow_line->speed_rot;

			return true;
		}

		return false;

	} break;

	case TASK_INTAKE_PULLER: 
	{
		robot->intake.intake_speed = CFG_INTAKE_MAX_SPEED;
		if(robot->intake.beam_break_val == 0)
		{
			robot->intake.intake_speed = 0;
			return true;
		}
		return false;
    
    } break;

	case TASK_INTAKE_WITHOUT_BB: 
	{

		float intake_speed = robot->input.mate.trigger_right;
		if(robot->input.mate.trigger_right < 0.5) intake_speed = 0.5;

		robot->intake.intake_speed = intake_speed;
		if(robot->input.mate.trigger_right < 0.05)
		{
			robot->intake.intake_speed = 0;
			return true;
		}
		return false;
    
    } break;

	case TASK_SHOOTER_POSITIONING: 
	{
		robot->shooter.target_angle = task->shooter.target_angle;
		bool angle_complete = ( fabsf(robot->shooter.target_angle - robot->shooter.sum_angle) < task->shooter.epsilon );

		return angle_complete;
	} break;

	case TASK_SHOOTER_PULLER: 
	{
		robot->shooter.control_motor_speed = CFG_SHOOTER_CONTROL_MAX_SPEED;
		robot->intake.intake_speed = CFG_INTAKE_MAX_SPEED;

		if(robot->shooter.beam_break_val == 0)
		{
			robot->shooter.control_motor_speed = 0;
			robot->intake.intake_speed = 0;

			return true;
		}
		return false;
    
    } break;

	default: break;
	}

	
	
	// returns true on complete
	return true;
}

///



bool pushTask(TaskMgr* mgr, Task task) {
	if (((mgr->write_head + 1) % TASKMGR_MAX_TASKS) == mgr->read_head) {
		printf("TaskMgr; Failed to add task, queue is full!\n");
		return false;
	}

	mgr->task_buffer[mgr->write_head] = task;
	mgr->write_head = (mgr->write_head + 1) % TASKMGR_MAX_TASKS;
	return true;
}




Task genTaskList(TaskMgr* list) {
	Task t;
	t.type = TASK_LIST;
	t.list = (TaskMgr*)malloc(sizeof(TaskMgr));
	*t.list = *list;
	return t;
}