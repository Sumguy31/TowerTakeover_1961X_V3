#include "main.h"
//////////////////////////////////////////////////////////////////////////////
//								Declarations								//
//////////////////////////////////////////////////////////////////////////////

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::Motor driveLeftFront(20, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200
	pros::Motor driveLeftBack(19, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_DEGREES);//200
	pros::Motor driveRightFront(15, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_DEGREES);//200
	pros::Motor driveRightBack(14, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200

	pros::Motor intake_1(2, pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200,right
	pros::Motor intake_2(10, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_DEGREES);//200
	
	pros::Motor tilt(9, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_DEGREES);//100
	
	pros::Motor arm(1,pros::E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);//200



	int iTimeCount;
	float powerAdjust = 1;
	bool left = false;

	int dArmMed = 620;
	int dArmLow = 470;

	int tiltHalf = 2560;

	bool safety = false;
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
	}
}
void on_right_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		left = false;
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
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
	while(pros::competition::is_disabled){
		pros::lcd::register_btn0_cb(on_left_button);
		pros::lcd::register_btn2_cb(on_right_button);
		if(left){
			pros::lcd::set_text(5,"Autonoumous Selected: LEFT");
		}else{
			pros::lcd::set_text(5,"Autonoumous Selected: RIGHT");
		}
		pros::delay(10);
	}
}

//////////////////////////////////////////////////////////////////////////////////
//									Auton Functions								//
//////////////////////////////////////////////////////////////////////////////////

//returns average right encoder value
int getRightEncoder(){
	return driveRightBack.get_position() + driveRightFront.get_position();
}
//returns average left encoder value
int getLeftEncoder(){
	return driveLeftBack.get_position() + driveLeftFront.get_position();
}
//returns average encoder value of all base motors
int getAvgEncoder(){
	return getLeftEncoder() + getRightEncoder();
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
int inchesToDegrees(int inches){
	return (inches/(M_PI*3.25))*720*(5/3);
}
//Set all drive motors to zero
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
/**Ramp up right side velocity 
 * Suggested:
 * Max Velocity: 130 rpm
 * acceleration: ???
 * interval: 10 milliseconds
 * 
 * NOTE: no need to put a negative sign when ramping down
 */
void rampRight(int MaxVoltage, int acceleration, int interval, bool up){
	int startVoltage;
	if(up){
		startVoltage = 50;
		int currentVoltage = startVoltage;
		while( abs( driveRightFront.get_voltage() )< MaxVoltage){
			driveRightFront.move_voltage(currentVoltage);
			driveRightBack.move_voltage(currentVoltage);
			currentVoltage += acceleration;
			pros::delay(interval);
		}
	}else{
		startVoltage = driveRightBack.get_voltage();
		int currentVoltage = startVoltage;
		while( abs( driveRightFront.get_voltage() ) > MaxVoltage){
			driveRightFront.move_voltage(currentVoltage);
			driveRightBack.move_voltage(currentVoltage);
			currentVoltage -= acceleration;
			pros::delay(interval);
		}
	}

}
/**Ramp up left side velocity 
 * Suggested:
 * Max Velocity: 130 rpm
 * acceleration: ???
 * interval: 10 milliseconds
 * 
 * NOTE: no need to put a negative sign when ramping down
 */
void rampLeft(int MaxVelocity, int acceleration, int interval, bool up){
	int startVelocity;	
	if(up){
		int currentVelocity = startVelocity;
		while( abs( driveLeftFront.get_actual_velocity() )< MaxVelocity){
			driveLeftFront.move_velocity(currentVelocity);
			driveLeftBack.move_velocity(currentVelocity);
			currentVelocity += acceleration;
			pros::delay(interval);
		}
	}else{
		acceleration = -acceleration;
		int startVelocity = driveRightBack.get_actual_velocity();
		int currentVelocity = startVelocity;
		while( abs( driveRightFront.get_actual_velocity() ) > MaxVelocity){
			driveLeftFront.move_velocity(currentVelocity);
			driveLeftBack.move_velocity(currentVelocity);
			currentVelocity += acceleration;
			pros::delay(interval);
		}
	}
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
		pros::lcd::print(5, "power: %d\n", power);
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
 *10 appears to be 185 degrees
 *5 appears to be 93 degrees
 *2.5 appears to be 45 degrees
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
void turnS(float Xinches, float Yinches, int maxVoltage, bool right){
	int startTime = iTimeCount;
	int halfTime = 350;
	int halfVoltage = maxVoltage*0.9;
	Yinches = Yinches/2;
	move(Yinches,-150,false);
	if(right){
		driveLeftFront.move_voltage(-halfVoltage);
		driveLeftBack.move_voltage(-halfVoltage);
		driveRightFront.move_voltage(0);
		driveRightBack.move_voltage(0);	
		pros::delay(halfTime);
		pros::lcd::set_text(4, "1st turn Complete");
		move(Xinches,-150,false);
		pros::lcd::set_text(5, "Reverse Complete");
		driveLeftBack.move_voltage(0);
		driveLeftFront.move_voltage(0);
		driveRightBack.move_voltage(-halfVoltage);
		driveRightFront.move_voltage(-halfVoltage);
		pros::delay(halfTime);
		pros::lcd::set_text(6, "2nd turn Complete");
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
	move(Yinches,-150,true);
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
//runs cube placement routine
void placeCube(){
	tilt.move_voltage(12000);
	pros::delay(1500);
	move(1.25,25);
	setIntake(-10000);
	pros::delay(500);
	move(20,-150);
}
/**Task counts seconds for autonous
 * 10 Ticks every 10 milliseconds
 */ 
void countTime(void *t){
	iTimeCount = 0;
	while(pros::competition::is_autonomous()){
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
/** runs deploy sequence
 *  designed to be run during the autonomous period
 */
void deploy(){
	setLeftDriveV(-3000);
	setRightDriveV(-3000);

	setIntake(-7000);
	moveArm(dArmMed, 6000);
	moveArm(0, 6000);
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
	setBrakeBrake();
	zeroDriveEncoder();
	zeroArm();
	pros::delay(100);
	deploy();


	/*moveArm(0,8000);
	setIntake(12000);
	movePID(25, 4000);
	
	pros::delay(750);
	movePID(-31,7000);
	/*setIntake(0);
	if(left){
		turn(9.4, 80);
	}else{
		turn(6.75, -80);
	}
	pros::delay(500);
	movePID(24, 8000);
	pros::delay(500);
	placeCube();
	/*movePID(-10,10000);
	turnS(30, 20,10000,true);
	movePID(70, 4000);
	pros::delay(1000);*/

	

	//turn(2.5, 80);
	pros::lcd::set_text(3, "complete!");


}

//////////////////////////////////////////////////////////////////////////////
//							Operator Control Tasks							//
//////////////////////////////////////////////////////////////////////////////

void auxControl(void* x){
	zeroArm();
	bool bArmUp = false;
	while(true){
		if(master.get_digital(DIGITAL_R1) && !bArmUp){
			moveArm(dArmMed, 12000);
			bArmUp = true;
			setArm(1000);
		}else if(master.get_digital(DIGITAL_R2) && !bArmUp){
			moveArm(dArmLow, 12000);
			bArmUp = true;
			setArm(1000);
		}else if( bArmUp && (master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_R1)) ){
			moveArm(0, 12000);
			bArmUp = false;
		}
		pros::delay(20);

		if(master.get_digital(DIGITAL_UP)){
			moveTilt(tiltHalf, 12000);
			moveTilt(5900,6000);
		}else{
			tilt.move(master.get_analog(ANALOG_LEFT_Y));
		}
	}
}

void diaganosticControl(void*x){
	while(true){
		if(master.get_digital(DIGITAL_A)){
			setArm(-8000);
			pros::delay(100);
			while(master.get_digital(DIGITAL_A) && !safety){
				pros::delay(5);
			}
			setArm(0);
			zeroArm();
		}
		safety = master.get_digital(DIGITAL_B);

		pros::lcd::print(4,"Safety: %d", safety);
		pros::lcd::print(5,"Arm Position %f", arm.get_position());
		pros::delay(10);
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
	pros::Task controlAux(auxControl, NULL, "Tilt/Intake Control System");
	pros::Task controlDiaganostic(diaganosticControl, NULL, "Diaganostic System");
	while (true) {
		int forward;
		int turn;

		forward = master.get_analog(ANALOG_RIGHT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X);
		setLeftDriveV((forward + turn) *100);
		setRightDriveV((forward - turn) * 100 * powerAdjust);

		pros::delay(20);

		if(master.get_digital(DIGITAL_L1)){
			setIntake(12000);
		}else if(master.get_digital(DIGITAL_L2)){
			setIntake(-12000);
		}else{
			setIntake(0);
		}
	}
}