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
	// printf("Currently Completing Task %d\n", task->type);
	if (complete) {
		printf("Task Complete %d\n", task->type);
		if (task->type == TASK_LIST) free(task->list);
		*task = { };
		mgr->read_head = (mgr->read_head + 1) % TASKMGR_MAX_TASKS;
	}
}

void updateManager(TaskMgr* mgr, RobotData* robot) {

	if(mgr->write_head > mgr->read_head)
	{
		// printf("Task Manager Num Tasks = %d \n", mgr->write_head - mgr->read_head);
	}

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

		v2 translation = robot->localiser.pose_estimate.position - task->waypoint.target_pose.position;
		frc::SmartDashboard::PutNumber("task translation x", translation.x);
		frc::SmartDashboard::PutNumber("task translation y", translation.y);

		return false;

	} break;

	case TASK_DRIVETRAIN_VELOCITY: 
	{
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
		float intake_speed = 0;

		if(robot->input.mate.trigger_right > 0) intake_speed = robot->input.mate.trigger_right;
		else if(robot->input.mate.trigger_left > 0) intake_speed = -robot->input.mate.trigger_left;

		robot->intake.intake_speed = intake_speed;

		if(robot->input.mate.trigger_right < 0.05 && robot->input.mate.trigger_left < 0.05)
		{
			robot->intake.intake_speed = 0;
			return true;
		}
		return false;
    
    } break;

	case TASK_SHOOTER_POSITIONING: 
	{
		// printf("Setting Angle\n");
		robot->shooter.target_angle = task->shooter.target_angle;
		float curr_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE;
		bool angle_complete = ( fabsf(robot->shooter.target_angle - curr_angle) < task->shooter.epsilon );
		if (angle_complete)
		{
			printf("Position Achieved\n");
		}
		// printf("NOT COMPLETE delta = %f\n", fabsf(robot->shooter.target_angle - curr_angle));
		return angle_complete;
	} break;

	case TASK_SHOOTER_PULLER: 
	{
		robot->shooter.control_motor_speed = CFG_SHOOTER_CONTROL_MAX_SPEED;
		robot->intake.intake_speed = CFG_INTAKE_MAX_SPEED;

		if(!robot->input.mate.a.held)
		{
			robot->shooter.control_motor_speed = 0;
			robot->intake.intake_speed = 0;

			return true;
		}
		return false;
    
    } break;

	case TASK_SHOOTER_FIRE:
	{
		robot->shooter.firing_motor_speed = -CFG_SHOOTER_MAX_FIRING_SPEED;
		if(task->firing_motor.direction != NULL) robot->shooter.firing_motor_speed *= task->firing_motor.direction;
		robot->shooter.firing_motor_task = true;
		robot->shooter.firing_mode = true;
		printf("Shooter Fire\n");
		return true;
	} break;

	case TASK_SHOOTER_STOP:
	{
		robot->shooter.firing_motor_speed = 0;
        robot->shooter.brake = true;

		//Im prob making duplicates of bools but i cant remember XD
		robot->shooter.firing_motor_task = false;
		robot->shooter.shooter_first_time = true;
		robot->shooter.firing_mode = false;
		printf("Shooter Stop\n");
		return true;
	} break;

	case TASK_SEAT_RING: 
	{
		if(task->shooter.seat_speed_firing != 0) robot->shooter.firing_motor_speed = task->shooter.seat_speed_firing;
		if(task->shooter.seat_speed_control != 0) 
		{
			printf("seating shooter \n");
			robot->shooter.control_motor_speed = task->shooter.seat_speed_control;
		}

		task->shooter.delay_timer += robot->delta_time;
		bool task_complete = false;
		robot->shooter.intake_task = false;
		if ( task->shooter.delay_timer > task->shooter.delay_length)
		{
			task_complete = true;
			robot->shooter.control_motor_speed = 0;
		}

		return task_complete;
	} break;

	case TASK_ANGLE_TO_TAG:
	{
		// Aim at tag

		bool task_complete = false;
		// printf("n_tags = %d", robot->photon.n_tags);

		if(robot->photon.n_tags != 0)
		{
			float calculated_throttle = 0;
			float tag_y_dist = static_cast<float>(robot->photon.tag_rel_robot[task->photon_aligner.align_tag_id - 1].Y());
			calculated_throttle = 2 * evalPid(&robot->drivetrain_controller.tag_aligner_pid, tag_y_dist, CFG_DELTA_TIME);

			frc::SmartDashboard::PutNumber("Tag Calculated Y Dist", tag_y_dist);

			calculated_throttle = CLAMP(calculated_throttle, -CFG_MAX_TAG_ALIGN_THROTTLE, CFG_MAX_TAG_ALIGN_THROTTLE);

			// task->photon_aligner.angular_throttle = mix(
        	// task->photon_aligner.angular_throttle,
        	// calculated_throttle,
        	// 0.2f
    		// );
			task->photon_aligner.angular_throttle = calculated_throttle;

			// printf("calculated throttle = %f\n", angular_throttle);
		}
		else task->photon_aligner.angular_throttle = 0;

		robot->drivetrain_controller.mode = DRIVECTRL_THROTTLE;
		robot->drivetrain_controller.ctrl.throttle.throttle = robot->global_input_translation;
		robot->drivetrain_controller.ctrl.throttle.angular_throttle = task->photon_aligner.angular_throttle; // Uncomment to enable robot rotational movement


		//Projectile Motion
		v2 vect_to_tag = {static_cast<float>(robot->photon.tag_rel_robot[task->photon_aligner.align_tag_id - 1].Y()), static_cast<float>(robot->photon.tag_rel_robot[task->photon_aligner.align_tag_id - 1].X())};
		float dist_from_tag = length(vect_to_tag);

		// float init_velocity = 11.276; // m/s Not constant, possibly make relative to shooters calculated speed
		float init_velocity = 0.00195305 * robot->shooter.firing_encoder->GetVelocity() + 1.49364;

		// Uncomment after we see goodish results
		float shooter_curr_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;
		float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_curr_angle ) + CFG_SHOOTER_AXIS_HEIGHT;

		// float shooter_offset = CFG_SHOOTER_DIST_CAM_TO_AXIS - CFG_SHOOTER_RADIUS * cosf(shooter_curr_angle);
		// dist_from_tag += shooter_offset;

		float equation_term_1 = (CFG_GRAVITATIONAL_CONSTANT * std::pow(dist_from_tag, 2)) / std::pow(init_velocity, 2);

		float solved_angle_1 = atan( (dist_from_tag - fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );
		float solved_angle_2 = atan( (dist_from_tag + fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );

		float solved_shooter_angle = (solved_angle_1 < solved_angle_2) ? solved_angle_1 : solved_angle_2;

		if (isnanf(solved_shooter_angle) == 0)
		{
			robot->shooter.target_angle = solved_shooter_angle - CFG_SHOOTER_ANGLE_OFFSET; // Uncomment to enable shooter a movement
		}

		frc::SmartDashboard::PutNumber("Calculated Angle", solved_shooter_angle);

		// printf("Projectile Motion Solved Angle = %f\n", solved_shooter_angle);



		if(robot->input.mate.trigger_left < 0.01f) 
		{
			robot->photon.first_aim = true;
			task_complete = true;
		}

		return task_complete;
	}break;

	case TASK_ANGLE_TO_TAG_AUTO:
	{
		// Aim at tag

		// printf("n_tags = %d", robot->photon.n_tags);

		bool aim_at_tag = false;

		if(robot->photon.n_tags != 0)
		{
			float calculated_throttle = 0;
			float tag_y_dist = static_cast<float>(robot->photon.tag_rel_robot[task->photon_aligner.align_tag_id - 1].Y());
			calculated_throttle = 2 * evalPid(&robot->drivetrain_controller.tag_aligner_pid, tag_y_dist, CFG_DELTA_TIME);

			// printf("pid = %f\n", calculated_throttle);

			calculated_throttle = CLAMP(calculated_throttle, -CFG_MAX_TAG_ALIGN_THROTTLE, CFG_MAX_TAG_ALIGN_THROTTLE);

			// task->photon_aligner.angular_throttle = mix(
        	// task->photon_aligner.angular_throttle,
        	// calculated_throttle,
        	// 0.2f
    		// );
			task->photon_aligner.angular_throttle = calculated_throttle;

			if(tag_y_dist < 0.1) aim_at_tag = true;

			// printf("calculated throttle = %f\n", angular_throttle);
		}
		else task->photon_aligner.angular_throttle = 0;

		robot->drivetrain_controller.mode = DRIVECTRL_THROTTLE;
		robot->drivetrain_controller.ctrl.throttle.throttle = robot->global_input_translation;
		robot->drivetrain_controller.ctrl.throttle.angular_throttle = task->photon_aligner.angular_throttle; // Uncomment to enable robot rotational movement


		//Projectile Motion
		v2 vect_to_tag = {static_cast<float>(robot->photon.tag_rel_robot[task->photon_aligner.align_tag_id - 1].Y()), static_cast<float>(robot->photon.tag_rel_robot[task->photon_aligner.align_tag_id - 1].X())};
		float dist_from_tag = length(vect_to_tag);

		// float init_velocity = 11.276; // m/s Not constant, possibly make relative to shooters calculated speed
		float init_velocity = 0.00195305 * robot->shooter.firing_encoder->GetVelocity() + 1.49364;

		// Uncomment after we see goodish results
		float shooter_total_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE + CFG_SHOOTER_ANGLE_OFFSET;
		float shooter_height = CFG_SHOOTER_RADIUS * sinf( shooter_total_angle ) + CFG_SHOOTER_AXIS_HEIGHT;

		// float shooter_offset = CFG_SHOOTER_DIST_CAM_TO_AXIS - CFG_SHOOTER_RADIUS * cosf(shooter_total_angle);
		// dist_from_tag += shooter_offset;

		float equation_term_1 = (CFG_GRAVITATIONAL_CONSTANT * std::pow(dist_from_tag, 2)) / std::pow(init_velocity, 2);

		float solved_angle_1 = atan( (dist_from_tag - fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );
		float solved_angle_2 = atan( (dist_from_tag + fabs( sqrtf( std::pow(dist_from_tag, 2) - 2 * equation_term_1 * ( 1/2 * equation_term_1 + CFG_SPEAKER_HEIGHT - shooter_height) ) ) ) / equation_term_1 );

		float solved_shooter_angle = (solved_angle_1 < solved_angle_2) ? solved_angle_1 : solved_angle_2;

		if (isnanf(solved_shooter_angle) == 0)
		{
			robot->shooter.target_angle = solved_shooter_angle - CFG_SHOOTER_ANGLE_OFFSET; // Uncomment to enable shooter a movement
		}

		float shooter_curr_angle = robot->shooter.sum_angle / CFG_SHOOTER_MAX_ANGLE * CFG_SHOOTER_ANGLE_RANGE;

		// printf("Projectile Motion Solved Angle = %f\n", solved_shooter_angle);

		bool final_task = false;

		bool task_complete = (robot->shooter.target_angle - shooter_curr_angle) < task->photon_aligner.shooter_align_epsilon;

		if(task_complete && task->photon_aligner.timer_first && aim_at_tag)
		{
			task->photon_aligner.timer = 0;
			task->photon_aligner.timer_first = false;
		}
		else if(task_complete && aim_at_tag)
		{
			task->photon_aligner.timer += CFG_DELTA_TIME;
		}

		if(task->photon_aligner.timer > 4.0f && task_complete && aim_at_tag)
		{
			final_task = true;
		}
			
		return final_task;
	}break;

	case TASK_ELEVATOR_POSITIONING: 
    {
        // printf("Setting Angle\n");
        robot->elevator.target_height = task->elevator.target_height;
        float curr_angle = robot->elevator.sum_rotation / CFG_ELEVATOR_MAX_ROTATION * CFG_ELEVATOR_RANGE;
        bool height_complete = ( fabsf(robot->elevator.target_height - curr_angle) < task->elevator.epsilon );
        if (height_complete)
        {
            printf("Position Achieved\n");
        }
        printf("NOT COMPLETE delta = %f\n", fabsf(robot->elevator.target_height - curr_angle));
        return height_complete;
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

	printf("Task Pushed %d\n", task.type);

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