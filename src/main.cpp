#include "main.h"
//////////////////////////////////////////////////////////////////////////////
//								Declarations								//
//////////////////////////////////////////////////////////////////////////////

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::Motor driveLeftFront(20, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200
	pros::Motor driveLeftBack(19, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200
	pros::Motor driveRightFront(15, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_DEGREES);//200
	pros::Motor driveRightBack(14, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_DEGREES);//200

	pros::Motor intake_1(2, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200,right
	pros::Motor intake_2(10, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_DEGREES);//200
	
	pros::Motor tilt(9, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_DEGREES);//100
	
	pros::Motor arm(1,pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200

	pros::ADILineSensor LineSensor('h');

	int iTimeCount;
	float powerAdjust = 1;


	int dArmMed = 700;
	int dArmLow = 490;

	int tiltHalf = 2560;

	bool safety = false;

	int autonNumb = 0;
	bool colorRed = true;
	bool left; 

	//arclenth measured in inches of the full arc
	float iArcLengthFull = 69.5;

	//Navigation
	int targetDistanceSmart;
	int	targetHeadingSmart;
	int leftTarget;
	int rightTarget;
	int maxVoltSmart = 12000;

	enum eDriveState {Turning, Moving, Waiting} driveState;
	enum eArmPosition {Bottom, Low, Med} armPosition;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_left_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		left = true;
		master.rumble("-");
	}
}
void on_right_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		left = false;
		//master.rumble("...");
	}
}
void autonSelect(){
	if(colorRed){
		master.print(2,0,"Color: Red");
	}else{
		master.print(2,0,"Color: Blue");
	}
	//while(!master.get_digital(DIGITAL_B)){
		if(master.get_digital(DIGITAL_UP)){
			while(master.get_digital(DIGITAL_UP)){
				pros::delay(10);
			}
			colorRed=!colorRed;
			if(colorRed){
				master.clear_line(2);
				pros::delay(50);
				master.print(2,0,"Color: Red");
			}else{
				master.print(2,0,"Color: Blue");
			}
		}
		pros::delay(10);
	//}
	left=!colorRed;
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	LineSensor.calibrate();
	pros::delay(500);
	pros::lcd::initialize();
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

//////////////////////////////////////////////////////////////////////////////////
//									Auton Functions								//
//////////////////////////////////////////////////////////////////////////////////

//returns average right encoder value
float getRightEncoder(){
	return 0.5*(driveRightBack.get_position() + driveRightFront.get_position());
}
//returns average left encoder value
float getLeftEncoder(){
	return 0.5*(driveLeftBack.get_position() + driveLeftFront.get_position());
}
//returns average encoder value of all base motors
float getAvgEncoder(){
	return 0.5*(getLeftEncoder() + getRightEncoder());
}
//zeros both left encoder values
void zero_left_encoder(){
	driveLeftBack.tare_position();
	driveLeftFront.tare_position();
}
//zeros both right encoder values
void zero_right_encoder(){
	driveRightBack.tare_position();
	driveRightFront.tare_position();
}
//zeros all drive enoder values
void zeroDriveEncoder(){
	zero_left_encoder();
	zero_right_encoder();
}
//sets drive to Brake Mode
void setBrakeBrake(){
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
//sets drive to Hold Mode
void setBrakeHold(){
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
//sets drive to Coast Mode
void setBrakeCoast(){
    driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
/**Waits until a target absolute time
 * Uses iTimeCount
 */
void waitUntil(int targetTime){
	while(iTimeCount < targetTime){
		pros::delay(10);
	}
}
//converts actual inches to rotational degrees
float inchesToDegrees(float inches){
	return (inches/(M_PI*3.25))*360.0*(3.0/5.0);
}
float degreesToInches(float degrees){
	return (degrees/360.0)*(5.0/3.0)*(M_PI*3.25);
}
/**converts arcmeasure to rotational degrees
 * Arcmeasure in degrees according to the Mathematical Convention:
 * ie. Left/CCW is positive and Right/CW is negative
 */
int arcmeasureToDegrees(float arcmeasure){
	return inchesToDegrees((arcmeasure/360.0)*iArcLengthFull)  ;
}
/**converts rotational degrees to arcmeasure
 * Arcmeasure in degrees according to the Mathematical Convention:
 * ie. Left/CCW is positive and Right/CW is negative
 */
int degreesToArcmeasure(float degrees){
	return (degreesToInches(degrees)/iArcLengthFull)*360.0;//check math
}
//returns current heading in arcmeasure degrees
int getHeading(){
	int heading = getRightEncoder() - getLeftEncoder();//get difference
	heading = degreesToArcmeasure(heading);//convert to arcmeasure degrees
	return heading;
}
//Set all drive motors to zero[0]
void stopDrive(){
	driveLeftFront.move_voltage(0);
	driveLeftBack.move_voltage(0);
	driveRightBack.move_voltage(0);
	driveRightFront.move_voltage(0);
}
/**sets voltage of right side drive
 * voltage from -12000 to 12000
 */
void setRightDriveV(int voltage){
	driveRightBack.move_voltage(voltage);
	driveRightFront.move_voltage(voltage);
}
/**sets voltage of left side drive
 * voltage from -12000 to 12000
 */
void setLeftDriveV(int voltage){
	driveLeftBack.move_voltage(voltage);
	driveLeftFront.move_voltage(voltage);
}
/**sets speed of right side drive
 * speed from -200 to 200
 */
void setRightDriveS(int speed){
	driveRightBack.move_velocity(speed);
	driveRightFront.move_velocity(speed);
}
/**sets speed of right side drive
 * speed from -200 to 200
 */
void setLeftDriveS(int speed){
	driveLeftFront.move_velocity(speed);
	driveLeftBack.move_velocity(speed);
}
int capNum(float num, float max, float min){
	num = num > max ? max : num;
	num = num < min ? min : num;
	return num;
}
//structure for PID Calculations
typedef struct {
    float current;
	float kP;
	float kI;
	float kD;
	float target;
	float integral;
	float error;
	float derivative;
	float lastError;
} PID;

PID sMovePid;
//calculate PID power
int iMovePid(int target){
    sMovePid.kP = 200;
    sMovePid.kI = 0;
    sMovePid.kD = 7000;

    sMovePid.current = getAvgEncoder();
    sMovePid.error = target - sMovePid.current;
    sMovePid.integral += sMovePid.error;
    sMovePid.derivative = sMovePid.error - sMovePid.lastError;
    sMovePid.lastError = sMovePid.error;

    return ( ((sMovePid.error)*(sMovePid.kP)) + ((sMovePid.derivative)*(sMovePid.kD)) + ((sMovePid.integral)*(sMovePid.kI)) );
}
//move with according to PID
void movePID(float distance){
    distance = inchesToDegrees(distance);
    int maxError = 2;
    int maxPower = 8000;
    int minVelocity = 10;
    int timer = 0;
    bool exit = false;

    zeroDriveEncoder();
    while( abs(getAvgEncoder() - distance) > maxError && !exit){
        int PIDPower = iMovePid(distance);
        int power = abs(PIDPower) < maxPower ? PIDPower : maxPower*(PIDPower/ abs(PIDPower));
		setLeftDriveV(power*powerAdjust);
		setRightDriveV(power);
        if(timer > 200 && abs(driveLeftFront.get_actual_velocity()) < minVelocity){
            exit = true;
        }
        pros::delay(10);
        timer +=10;
    }
 	stopDrive();
}
//move with according to PID
void movePID(float distance, int maxPower){
    distance = inchesToDegrees(distance);
    int maxError = 2;
    int minVelocity = 10;
    int timer = 0;
    bool exit = false;

    zeroDriveEncoder();
    while( abs(getAvgEncoder() - distance) > maxError && !exit){
        int PIDPower = iMovePid(distance);
        int power = abs(PIDPower) < maxPower ? PIDPower : maxPower*(PIDPower/ abs(PIDPower));
		setLeftDriveV(power*powerAdjust);
		setRightDriveV(power);
        if(timer > 200 && abs(driveLeftFront.get_actual_velocity()) < minVelocity){
            exit = true;
        }
        pros::delay(10);
        timer +=10;
    }
 	stopDrive();
}
PID sRightPID;
int iRightPID(float target){
	sRightPID.kP = 300;
    sRightPID.kI = 0;
    sRightPID.kD = 90000;

    sRightPID.current = getRightEncoder();
    sRightPID.error = target - sRightPID.current;
    sRightPID.integral += sRightPID.error;
    sRightPID.derivative = sRightPID.error - sRightPID.lastError;
    sRightPID.lastError = sRightPID.error;

    return ( ((sRightPID.error)*(sRightPID.kP)) + ((sRightPID.derivative)*(sRightPID.kD)) + ((sRightPID.integral)*(sRightPID.kI)) );
}

PID sLeftPID;
int iLeftPID(float target){
	sLeftPID.kP = 300;
    sLeftPID.kI = 0;
    sLeftPID.kD = 90000;

    sLeftPID.current = getLeftEncoder();
    sLeftPID.error = target - sLeftPID.current;
    sLeftPID.integral += sLeftPID.error;
    sLeftPID.derivative = sLeftPID.error - sLeftPID.lastError;
    sLeftPID.lastError = sLeftPID.error;

    return ( ((sLeftPID.error)*(sLeftPID.kP)) + ((sLeftPID.derivative)*(sLeftPID.kD)) + ((sLeftPID.integral)*(sLeftPID.kI)) );
}
PID sHeadingPID;
//calculate PID power for heading adjustment
int iHeadingPID(float target){
	sHeadingPID.kP = 400;
	sHeadingPID.kI = 1;
	sHeadingPID.kD = 10000;

	sHeadingPID.current = getHeading();
	sHeadingPID.error = target - sHeadingPID.current;
	sHeadingPID.integral += sHeadingPID.error;
	sHeadingPID.derivative = sHeadingPID.error - sHeadingPID.lastError;
	sHeadingPID.lastError = sHeadingPID.error;

	return ( (sHeadingPID.error*sHeadingPID.kP) + (sHeadingPID.derivative*sHeadingPID.kD) + (sHeadingPID.integral*sHeadingPID.kI) );
}
/**move with constant speed
 *
 *speed from -200 to +200
 */
void move(float inches, int speed){
	int degrees = inchesToDegrees(inches);
	zeroDriveEncoder();

	setLeftDriveS(speed*powerAdjust);
	setRightDriveS(speed);

	while(abs( getAvgEncoder() ) < degrees){
		pros::delay(10);
	}
	stopDrive();
}
/**move with constant speed
 *
 *speed from -200 to +200
 */
void move(float inches, int speed, bool brake){
	int degrees = inchesToDegrees(inches);
	zeroDriveEncoder();

	setLeftDriveS(speed*powerAdjust);
	setRightDriveS(speed);

	while(abs( getAvgEncoder() ) < degrees){
		pros::delay(10);
	}
	if(brake){
	stopDrive();
	}
}
/**turn with constant speed
 *
 *speed from -200 to +200
 *inches refers to the circumference of the circle
 *positive is ccw, negative is cw
 *
 *6 appears to be 185 degrees
 *3 appears to be 93 degrees
 *1 appears to be 45 degrees
 *
 *default speed is 80
 */
void turn(float inches, int speed){
	int degrees = inchesToDegrees(inches);
	zeroDriveEncoder();

	driveLeftBack.move_velocity(-speed);
	driveLeftFront.move_velocity(-speed);
	driveRightBack.move_velocity(speed);
	driveRightFront.move_velocity(speed);

	while(abs( getLeftEncoder() ) < degrees){
		pros::delay(10);
	}
	stopDrive();
}
/**Makes an S-turn movement Backwards
 * 
 * X-inches is inches moved latterally
 * Y-inches is inches moved longitudinally
 * Lat/Long motion is 150 rm
 * 
 * maxVoltage the maximum voltage from -12000 to 12000
 * boolean right determines whether the S is to the left or right
 */
void turnS(float Xinches, float Yinches, int maxVoltage, bool right, bool forward){
	int startTime = iTimeCount;
	int halfTime = 350;
	int halfVoltage = maxVoltage*1;
	Yinches = Yinches/2;
	if(forward){
		move(Yinches,150,false);
	}else{
		move(Yinches,-150,false);
	}
	
	if(right){
		if(forward){
			driveLeftFront.move_voltage(halfVoltage);
			driveLeftBack.move_voltage(halfVoltage);
			driveRightFront.move_voltage(0);
			driveRightBack.move_voltage(0);	
			pros::delay(halfTime);
			move(Xinches,150,false);
			driveLeftBack.move_voltage(0);
			driveLeftFront.move_voltage(0);
			driveRightBack.move_voltage(halfVoltage);
			driveRightFront.move_voltage(halfVoltage);
			pros::delay(halfTime+140);
		}else{
			driveLeftFront.move_voltage(-halfVoltage);
			driveLeftBack.move_voltage(-halfVoltage);
			driveRightFront.move_voltage(0);
			driveRightBack.move_voltage(0);	
			pros::delay(halfTime);
			move(Xinches,-150,false);
			driveLeftBack.move_voltage(0);
			driveLeftFront.move_voltage(0);
			driveRightBack.move_voltage(-halfVoltage);
			driveRightFront.move_voltage(-halfVoltage);
			pros::delay(halfTime);
		}
	}else{
		if(forward){
			driveRightBack.move_voltage(halfVoltage);
			driveRightFront.move_voltage(halfVoltage);
			driveLeftFront.move_voltage(0);
			driveLeftBack.move_voltage(0);
			pros::delay(halfTime);
			move(Xinches,150,false);
			driveRightBack.move_voltage(0);
			driveRightFront.move_voltage(0);
			driveLeftBack.move_voltage(halfVoltage);
			driveLeftFront.move_voltage(halfVoltage);
			pros::delay(halfTime);
		}else{
			driveRightBack.move_voltage(-halfVoltage);
			driveRightFront.move_voltage(-halfVoltage);
			driveLeftFront.move_voltage(0);
			driveLeftBack.move_voltage(0);
			pros::delay(halfTime);
			move(Xinches,-150,false);
			driveRightBack.move_voltage(0);
			driveRightFront.move_voltage(0);
			driveLeftBack.move_voltage(-halfVoltage);
			driveLeftFront.move_voltage(-halfVoltage);
			pros::delay(halfTime);
		}
	}
	if(forward){
		move(Yinches,150,false);
	}else{
		move(Yinches,-150,false);
	}

}


/**Smart drive fucntionality task
 * Holds and Adjusts Heading
 * Moves and based on PID
 * Turns based on PID
 * 
 * NOTE: Smart function does not yet account for differences in Encoder values
 * Encoders must be zeroed between each function. PID for each side is yet to be implemented
 */
void moveSmartAuto(void*){
	int forward, turn;
	int left, right;
	while(pros::competition::is_autonomous()){
		if(driveState == Turning){
			turn = iHeadingPID(targetHeadingSmart);//MAY NEED FIXING
			setLeftDriveV(-turn);
			setRightDriveV(+turn);
		}else if(driveState == Moving){
			forward = iMovePid(targetDistanceSmart);
			turn = iHeadingPID(targetHeadingSmart);

			left = iLeftPID(leftTarget);
			right = iRightPID(rightTarget);
			
			turn = capNum(turn, left, -left);
			turn = capNum(turn, right, -right);
			setLeftDriveV(capNum(left, maxVoltSmart, -maxVoltSmart) - turn);
			setRightDriveV(capNum(right, maxVoltSmart, -maxVoltSmart) + turn);
		}else if(driveState == Waiting){
			stopDrive();
		}else{
			return;//should never enter this... will edit to throw error later.
		}
	}
}
/**sets next move target for the robot
 * Distance measured in inches,
 * Heading measured in Arcmeasure degrees
 * Voltage from 0 to 12000
 */
void setMoveTarget( float distance, int heading, int maxVolt){
	zeroDriveEncoder();
	driveState = Moving;
	targetDistanceSmart = inchesToDegrees(distance);
	leftTarget = getLeftEncoder() + inchesToDegrees(distance);
	rightTarget = getRightEncoder() + inchesToDegrees(distance);
	targetHeadingSmart = heading;
	maxVoltSmart = maxVolt;
}
/**
 * 
 * 
 * 
 */
void setTurnTarget( float arcmeasure, int maxVolt){
	zeroDriveEncoder();
	driveState = Turning;
	targetHeadingSmart = arcmeasure;
	maxVoltSmart = maxVolt;
}
//waits for drive to reach destination
void waitForDrive(){
	float maxError = inchesToDegrees(0.25);//0.25 is max error in inches
	int minVel = 25;
	int i = 0;
	if(driveState = Turning){
		while(abs(getHeading() - targetHeadingSmart) < maxError && !safety){
			pros::delay(20);
			i += 20;
			safety = (i > 500 && 
					(abs(driveRightBack.get_actual_velocity()) < minVel || abs(driveLeftBack.get_actual_velocity()) < minVel));			
		}
	}else if(driveState = Moving){
		while(abs(getAvgEncoder() - targetDistanceSmart) < maxError && !safety){
			pros::delay(20);
			i += 20;
			safety = (i > 500 && 
					(abs(driveRightBack.get_actual_velocity()) < minVel || abs(driveLeftBack.get_actual_velocity()) < minVel));	
		}
	}else{
		return;
	}
	driveState = Waiting;
}
/**set intake using a voltage from -12000 to 12000
 * 
 * Positive is in
 * Negative is out
 */
void setIntake(int power){
	intake_1.move_voltage(power);
	intake_2.move_voltage(power);
}
/**Task counts seconds for autonous
 * 10 Ticks every 10 milliseconds
 */ 
void countTime(void *t){
	iTimeCount = 0;
	while(!pros::competition::is_disabled()){
		pros::delay(10);
		iTimeCount += 10;
	}
}
/**Sets Arm Voltage
 * from -12000 to 12000
 */
void setArm(int voltage){
	arm.move_voltage(voltage);
}
/**Zeros Arm position
 */
void zeroArm(){
	arm.tare_position();
}
/**Moves Arm to target position
 * voltage is measured from 0 to 12000
 */
void moveArm(int target, int voltage){
	int current = arm.get_position();
	int error = 30;
	if(current < target){
		setArm(voltage);
	}else{
		setArm(-voltage);
	}
	int difference = abs(current - target);
	while(difference > error && !safety){
		difference = abs( arm.get_position() - target);
		pros::delay(5);
	}

	setArm(0);
}
/**Moves tilt to target position
 * voltage is from 0 to 12000
 */
void moveTilt(int target, int voltage){
	int current = tilt.get_position();
	int error = 100;
	if(current < target){
		tilt.move_voltage(voltage);
	}else{
		tilt.move_voltage(-voltage);
	}
	int difference = abs(current - target);
	while(difference > error && !safety){
		difference = abs( tilt.get_position() - target);
		pros::delay(5);
	}

	tilt.move_voltage(0);
}
/**Centers cube using Light Sensor
 * Tolerance Default is 1200
 * uses calibrated setting
 * assumes sensor has successfully been calibrated
 */
void centerCube(){
	int tolerance = 2600;

	setIntake(-6000);
	while(LineSensor.get_value_calibrated() > tolerance && !safety){
		pros::delay(20);
	}
	pros::delay(300);
	setIntake(0);
}
/** runs deploy sequence
 *  designed to be run during the autonomous period
 */
void deploy(){
	setIntake(-12000);
	moveArm(dArmMed, 12000);
	pros::delay(200);
	moveArm(0, 9000);
}
//LCD program for autonomous
void LCDAuto(void*){
	pros::lcd::initialize();
	while(true){
		pros::delay(20);

		pros::lcd::print(5, "Heading: %i", getHeading());
		pros::lcd::print(6, "Inches: %f", degreesToInches(getAvgEncoder()) );
	}
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *inches
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::Task autoTime(countTime, NULL, "Count Milliseconds");
	pros::Task autoMove(moveSmartAuto, NULL, "Drive Task Autonmous");
	pros::Task autoLCD(LCDAuto, NULL, "LCD Task");
	left = !colorRed;
	
	
	if(pros::lcd::is_initialized()){
		setIntake(10000);
	}else{
		setIntake(-10000);
	}
	
	setMoveTarget(60, 0, 8000);
	waitForDrive();

	/*left = false;
	setBrakeBrake();
	zeroDriveEncoder();
	zeroArm();
	pros::delay(200);
	deploy();
	moveArm(0,8000);
	setArm(-4000);
	setIntake(12000);
	pros::delay(100);

	move(57, 75);

	pros::delay(750);
	setIntake(-2000);
	movePID(-47,7000);
	setIntake(0);
	if(left){
		turn(7.25, 60);
	}else{
		turn(7.25, -50);
	}
	setIntake(-4500);
	pros::delay(500);
	setIntake(0);
	move(11.5, 8000);
	setRightDriveV(4000);
	setLeftDriveV(4000);
	pros::delay(500);
	stopDrive();
	moveTilt(tiltHalf, 12000);
	moveTilt(5900,6000);
	setIntake(-12000);
	move(10,-200);*/

	pros::lcd::set_text(3, "complete!");


}

//////////////////////////////////////////////////////////////////////////////
//							Operator Control Tasks							//
//////////////////////////////////////////////////////////////////////////////

void auxControl(void* x){
	zeroArm();
	bool bArmUp = false;//Legacy code not needed
	armPosition = Bottom;
	setArm(-2000);
	int power;
	while(true){
		if(master.get_digital(DIGITAL_R1) && armPosition == Bottom){
			moveArm(dArmMed, 12000);
			bArmUp = true;//Legacy code not needed
			armPosition = Med;
			setArm(1000);
		}else if(master.get_digital(DIGITAL_R2) && armPosition == Bottom){
			moveArm(dArmLow, 12000);
			bArmUp = true;//Legacy code not needed
			armPosition = Low;
			setArm(1000);
		}else if( !(armPosition == Bottom) && (master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_R1)) ){
			moveArm(0, 12000);
			bArmUp = false;//Legacy code not needed
			armPosition = Bottom;
			setArm(-2000);
		}
		pros::delay(20);

		if(master.get_digital(DIGITAL_UP)){
			//moveTilt(tiltHalf, 12000);
			//moveTilt(5900,6000);
			power = 12000 - tilt.get_position();
			tilt.move_voltage(power);
		}else{
			tilt.move(master.get_analog(ANALOG_LEFT_Y));
		}

		if(safety && master.get_digital(DIGITAL_DOWN)){
			deploy();
		}
	}
}

void diaganosticControl(void*x){
	zeroDriveEncoder();
	while(true){

		if(master.get_digital(DIGITAL_A)){
			setArm(-8000);
			pros::delay(100);
			while(master.get_digital(DIGITAL_A) && !safety){
				pros::delay(5);
			}
			setArm(0);
			zeroArm();
			setArm(-2000);
		}
		safety = master.get_digital(DIGITAL_B);
		if(master.get_digital(DIGITAL_Y)){
			zeroDriveEncoder();
		}
		pros::delay(10);
		
		pros::lcd::print( 2, "Inches: %f", degreesToInches(abs(getLeftEncoder() - getRightEncoder())));
		pros::lcd::print( 3, "Difference: %i", abs(getLeftEncoder() - getRightEncoder()));
		pros::lcd::print( 4, "Left Encoder %f", getLeftEncoder());
		pros::lcd::print( 5, "Right Encoder %f", getRightEncoder());
		pros::lcd::print( 6, "Heading: %i", getHeading());
		pros::lcd::print( 7, "Light Sensor: %i", LineSensor.get_value_calibrated());

	}
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Task OpTime(countTime, NULL, "Count Milliseconds");
	pros::Task controlAux(auxControl, NULL, "Tilt/Intake Control System");
	pros::Task controlDiaganostic(diaganosticControl, NULL, "Diaganostic System");
	int forward;
	int turn;
	int intakePower;
	while (true) {
		forward = master.get_analog(ANALOG_RIGHT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X);
		setLeftDriveV((forward + turn) *100);
		setRightDriveV((forward - turn) * 100 * powerAdjust);

		intakePower = armPosition == Low ? 7000 : 12000;// if in low position intake power is 7000 else 12000

		if(master.get_digital(DIGITAL_L1)){
			setIntake(intakePower);
		}else if(master.get_digital(DIGITAL_L2)){
			setIntake(-intakePower);
		}else if(master.get_digital(DIGITAL_LEFT)){
			centerCube();
		}else{
			setIntake(0);
		}
		//pros::lcd::print(7, "Time (ms): %n", iTimeCount);
		pros::delay(10);
	}
}