/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

// Program States
typedef enum {
	INITIAL = 0, // Initial state
	PAUSE_WAIT_BUTTON_RELEASE, // Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS, // Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE, // Paused; pause button pressed down, wait until released before returning to previous state

	DRIVE, //move forwards
	BACK, //move backwards
	BUFFER, //encountered a slope, move forwards a bit to make mure the whole robot is on the slope for more accurate accel value measures
	BUFFER_PAUSE, //stop moving for a while to ensure stable readings from accelerometer
	TURN, //turn state, depends on the situation, will forwards to desired turn states
	REORIENT_RIGHT, //encountered a slope on the rhs, turn right to face the slope
	REORIENT_LEFT, //encountered a slope on the lhs, turn left to face the slope
	PAUSE, //after hitting the bottom of the ramp, stop moving
	AVOID_LEFT, //hit an obsticle on the right, turn left
	AVOID_RIGHT, //hit an obsticle on the left or center, turn right
	TURN_OPPOSITE, //decide whether to return left or return right after avoid left or avoiding right
	RETURN_LEFT, //avoided obsticle by turrning right, return to origional course by turnning left
	RETURN_RIGHT, //avoided obsticle by turning left, return to origional course by turnning right
	PARALLEL, //move sideways a certain distance to try to avoid the obsticle

	//second 90 degrees of turn 180 degrees
} robotState_t;


#define DEG_PER_RAD (180.0 / 3.14515) // degrees per radian
#define RAD_PER_DEG (M_PI / 180.0) // radians per degree
#define AVGCOUNT 25 //number readings to take an average
#define TILT_THRESHOLD 1
#define FALSE 0
#define TRUE 1
#define ACCEL_READING_THRESHOLD 0.0 //accelerometer is not accurate, compair readings within buffer range
#define KOBUKI_DIAMETER 100
#define int16_t short
#define int32_t int
#define int64_t long int
#define SPEED 250
#define REORIENT_TH 0.0169
#define FLAT_TH 0.12
//FLAGS
int32_t on_slope = FALSE;
int32_t cliffcenter = FALSE;
int32_t cliffleft = FALSE;
int32_t cliffright = FALSE;
int32_t bumpcenter = FALSE;
int32_t bumpright = FALSE;
int32_t bumpleft = FALSE;
int32_t turning = FALSE;
int32_t measure = FALSE;
int32_t reorient = FALSE;
int32_t interrupted = FALSE;
int32_t calibrate = TRUE;
int32_t direction_toggle = TRUE;
int32_t turnned_right = FALSE;
int32_t top_flat = FALSE;

//VARIABLES
double slope_direction;
double direction_before_slope;
int32_t angle_before_turn;
int32_t resume_angle;
int32_t desired_180;
int initial_angle;
double return_angle;
double stable_value;



double pitch;
double roll;
double yaw;
double offset_roll;
double offset_yaw;

double accelaverage_x;
double accelaverage_y;
double accelaverage_z;
double accel_offset_x;
double accel_offset_y;
double accel_offset_z;

int count;
double averagetotal_x, averagetotal_y, averagetotal_z;
double accel_y[AVGCOUNT];
double accel_x[AVGCOUNT];
double accel_z[AVGCOUNT];





void KobukiNavigationStatechart(const int16_t maxWheelSpeed, const int32_t netDistance, const int32_t netAngle,
	const KobukiSensors_t sensors, const accelerometer_t accelAxes, int16_t* const pRightWheelSpeed,
	int16_t* const pLeftWheelSpeed, const bool isSimulator)
{
	// local state
	static robotState_t state = INITIAL; // current program state
	static robotState_t unpausedState = DRIVE;
	static int32_t distanceAtManeuverStart = 0; // distance robot had travelled when a maneuver begins, in mm
	static int32_t angleAtManeuverStart = 0;
	// outputs
	int16_t leftWheelSpeed = 0; // speed of the left wheel, in mm/s
	int16_t rightWheelSpeed = 0; // speed of the right wheel, in mm/s


	accel_y[count] = accelAxes.y;
	accel_x[count] = accelAxes.x;
	accel_z[count] = accelAxes.z;
	int i;
	for (i = 0; i < AVGCOUNT; i++)
	{
		averagetotal_x += accel_x[i];
		averagetotal_y += accel_y[i];
		averagetotal_z += accel_z[i];
	}

	accelaverage_x = averagetotal_x / AVGCOUNT;
	accelaverage_y = averagetotal_y / AVGCOUNT;
	accelaverage_z = averagetotal_z / AVGCOUNT;

	count = count + 1;
	if (count == (AVGCOUNT - 1))
	{
		count = 0;
	}
	roll = DEG_PER_RAD * atan(accelaverage_y / (sqrt(accelaverage_x * accelaverage_x + accelaverage_z * accelaverage_z)));
	pitch = DEG_PER_RAD * atan(-accelaverage_x / accelaverage_z);
	yaw = DEG_PER_RAD * atan(accelaverage_x / accelaverage_y);


	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************
	if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0 // pause button
		) {
		switch (state) {
		case INITIAL:
			// set state data that may change between simulation and real-world
			if (isSimulator) {
			}
			else {
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0) {
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0) {
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0) {
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			else if (sensors.buttons.B1)
			{
				unpausedState = DRIVE;
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - run region      *
	//*************************************

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//encounter a bump
	else if (state == DRIVE && (sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight))
	{
		if (sensors.bumps_wheelDrops.bumpCenter)
		{
			bumpcenter = TRUE;
		}
		if (sensors.bumps_wheelDrops.bumpLeft)
		{
			bumpleft = TRUE;
		}
		if (sensors.bumps_wheelDrops.bumpRight)
		{
			bumpright = TRUE;
		}
		distanceAtManeuverStart = netDistance;
		measure = FALSE;
		state = BACK;
	}

	//encounter a cliff
	else if (state == DRIVE && (sensors.cliffCenter || sensors.cliffLeft || sensors.cliffRight))
	{
		if (sensors.cliffCenter)
		{
			cliffcenter = TRUE;
			reorient = FALSE;
		}
		else if (sensors.cliffLeft)
		{
			cliffleft = TRUE;
		}
		else if (sensors.cliffRight)
		{
			cliffright = TRUE;
		}
		measure = FALSE;
		distanceAtManeuverStart = netDistance;
		state = BACK;
	}


	//encounter a slope
	else if ((state == DRIVE) && !measure && !(sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight) && (fabs(accelAxes.x) > 0.12))
	{
		on_slope = TRUE;
		reorient = TRUE;
		direction_before_slope = netAngle;
		distanceAtManeuverStart = netDistance;
		if (top_flat == TRUE)
		{
			measure = TRUE;
		}
		state = BUFFER;
	}

	//on the slope and detects flat ground
	else if ((state == DRIVE) && (on_slope == TRUE) && (fabs(accelAxes.x) <= FLAT_TH)) //&& (measure == TRUE))
	{
		on_slope = FALSE;
		reorient = FALSE;
		if (top_flat == FALSE)
		{
			top_flat = TRUE;
			state = DRIVE;
		}
		else
		{
			measure = FALSE;
			state = PAUSE;
		}
	}


	else if (state == BACK && abs(netDistance - distanceAtManeuverStart) >= 50)
	{
		angle_before_turn = netAngle;
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN;
	}


	else if ((state == BUFFER) && (abs(netDistance - distanceAtManeuverStart) >= KOBUKI_DIAMETER))
	{
		//state = REORIENT_RIGHT;
		if (top_flat == FALSE)
		{
			if (accelAxes.y < -0.01)
			{
				state = REORIENT_RIGHT;
			}
			else if (accelAxes.y > 0.01)
			{
				state = REORIENT_LEFT;
			}
		}
		else
		{
			if (accelAxes.y > -0.01)
			{
				state = REORIENT_RIGHT;
			}
			else if (accelAxes.y < 0.01)
			{
				state = REORIENT_LEFT;
			}

		}
	}


	//reorient to face the slope using accel_y values
	else if ((state == REORIENT_LEFT) && (reorient == TRUE) && (fabs(accelAxes.y) <= REORIENT_TH))
	{
		reorient = FALSE;
		slope_direction = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}

	else if ((state == REORIENT_LEFT) && (sensors.cliffCenter || sensors.cliffLeft || sensors.cliffRight))
	{
		if (sensors.cliffCenter)
		{
			cliffcenter = TRUE;
			reorient = FALSE;
		}
		else if (sensors.cliffLeft)
		{
			cliffleft = TRUE;
		}
		else if (sensors.cliffRight)
		{
			cliffright = TRUE;
		}
		measure = FALSE;
		distanceAtManeuverStart = netDistance;
		state = BACK;
	}
	else if ((state == REORIENT_LEFT) && (sensors.bumps_wheelDrops.wheeldropRight))
	{
		state = DRIVE;
	}

	else if ((state == REORIENT_LEFT || state == REORIENT_RIGHT) && (fabs(accelAxes.x) <= FLAT_TH))
	{
		reorient = FALSE;
		if (on_slope == TRUE)
		{
			top_flat = TRUE;
			state = DRIVE;
		}
		else
		{
			state = PAUSE;
		}
	}

	else if ((state == REORIENT_RIGHT) && (reorient == TRUE) && (fabs(accelAxes.y) <= REORIENT_TH))
	{
		//reorient = FALSE;
		slope_direction = netAngle;
		reorient = FALSE;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}
	else if ((state == REORIENT_RIGHT) && (sensors.cliffCenter || sensors.cliffLeft || sensors.cliffRight))
	{
		if (sensors.cliffCenter)
		{
			cliffcenter = TRUE;
			reorient = FALSE;
		}
		else if (sensors.cliffLeft)
		{
			cliffleft = TRUE;
		}
		else if (sensors.cliffRight)
		{
			cliffright = TRUE;
		}
		measure = FALSE;
		distanceAtManeuverStart = netDistance;
		state = BACK;
	}
	else if ((state == REORIENT_RIGHT) && (sensors.bumps_wheelDrops.wheeldropLeft))
	{
		state = DRIVE;
	}

	
	//hit an obsticle infront, turn right
	else if (state == TURN && ((bumpcenter == TRUE) || (cliffcenter == TRUE)))
	{
		bumpcenter = FALSE;
		cliffcenter = FALSE;
		angle_before_turn = netAngle;
		if (interrupted == TRUE)
		{
			interrupted = FALSE;
			state = AVOID_LEFT;
		}
		else {

			state = AVOID_RIGHT;
		}
	}

	//hit an obsticle on the left, turn right
	else if (state == TURN && ((bumpleft == TRUE) || (cliffleft == TRUE)))
	{
		bumpleft = FALSE;
		cliffleft = FALSE;
		angle_before_turn = netAngle;
		state = AVOID_RIGHT;
	}

	//hit an obsticle on the right, turn left
	else if (state == TURN && ((bumpright == TRUE) || (cliffright == TRUE)))
	{
		cliffright = FALSE;
		bumpright = FALSE;
		angle_before_turn = netAngle;
		state = AVOID_LEFT;
	}


	//avoided the obsticle by turrning right, now move sideways to clear the obsticle
	else if (state == AVOID_RIGHT && (abs(netAngle - angle_before_turn) >= 88))
	{
		distanceAtManeuverStart = netDistance;
		turnned_right = TRUE;
		state = PARALLEL;
	}

	//avoided the obsticle by turnning left, now move sideways to clear the obsticle
	else if (state == AVOID_LEFT && (abs(netAngle - angle_before_turn) >= 88))
	{
		distanceAtManeuverStart = netDistance;
		state = PARALLEL;
	}

	//move side ways for a set distance to clear the obsticle
	else if (state == PARALLEL && (abs(netDistance - distanceAtManeuverStart) > 350))
	{
		if (on_slope)
		{
			return_angle = slope_direction;
		}
		else
		{
			return_angle = 0;
		}
		state = TURN_OPPOSITE;
	}


	//after moving side ways, turn back to face the origional direction of travel
	else if (state == TURN_OPPOSITE)
	{
		if (turnned_right)
		{
			state = RETURN_LEFT;
		}
		else
		{
			state = RETURN_RIGHT;
		}

	}

	//avoided the obsticle by turrning right, turn left again to face the origional direaction
	else if (state == RETURN_LEFT && (netAngle - return_angle >= 1))
	{
		turnned_right = FALSE;
		interrupted = FALSE;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}

	//avoided the obsticle by turning left, turn right again to face the origional direction
	else if (state == RETURN_RIGHT && (netAngle - return_angle <= 1))
	{
		interrupted = FALSE;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}

	
	


	//*****************
	//* state actions *
	//*****************
	switch (state) {
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		leftWheelSpeed = SPEED;
		rightWheelSpeed = SPEED;
		break;

	case PARALLEL:
		leftWheelSpeed = SPEED;
		rightWheelSpeed = SPEED;
		break;

	case BUFFER:
		leftWheelSpeed = SPEED;
		rightWheelSpeed = SPEED;
		break;

	case BUFFER_PAUSE:
		leftWheelSpeed = 0;
		rightWheelSpeed = 0;
		break;

	case BACK:
		leftWheelSpeed = -SPEED;
		rightWheelSpeed = -SPEED;
		break;

	case AVOID_LEFT:
		leftWheelSpeed = -SPEED;
		rightWheelSpeed = SPEED;
		break;

	case AVOID_RIGHT:
		leftWheelSpeed = SPEED;
		rightWheelSpeed = -SPEED;
		break;

	case RETURN_LEFT:
		leftWheelSpeed = -SPEED;
		rightWheelSpeed = SPEED;
		break;

	case RETURN_RIGHT:
		rightWheelSpeed = -SPEED;
		leftWheelSpeed = SPEED;
		break;

	case REORIENT_RIGHT:
		leftWheelSpeed = SPEED;
		rightWheelSpeed = SPEED - 50;
		break;

	case REORIENT_LEFT:
		leftWheelSpeed = SPEED - 50;
		rightWheelSpeed = SPEED;
		break;

	case TURN_180_RIGHT:
		leftWheelSpeed = -SPEED;
		rightWheelSpeed = SPEED;
		break;
	case TURN_180_LEFT:
		leftWheelSpeed = SPEED;
		rightWheelSpeed = -SPEED;
		break;
	case PAUSE:
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}


	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
