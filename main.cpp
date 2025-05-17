#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup left_motors({-2, -3, -4}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({9, 10, 8}, pros::MotorGearset::blue);
//pros::Imu imu(4);
pros::Motor intake(-11, pros::MotorGearset::blue);
pros::Motor leftWallStake(12, pros::MotorGearset::blue);
pros::Motor rightWallStake(-13, pros::MotorGearset::blue);
//pros::adi::Pneumatics clamp1("A");
//pros::adi::Pneumatics clamp2("H");
//pros::adi::Pneumatics clamp1 = pros::adi::Pneumatics('A'); // Create a pneumatic in port A
//pros::adi::digital clamp1('A'); // Create a pneumatic in port A
pros::adi::DigitalOut clamp1('A', false);
pros::adi::DigitalOut clamp2('B', false);
pros::adi::DigitalOut wall_stake_piston('F', false);
pros::Imu imu(6);

volatile int controlMode = 0; // 0 = None, 1 = Lift Control, 2 = Alliance Control
void setControlMode(int mode) {
    controlMode = mode;
}

const int numWallStates = 3;
int wallStates[numWallStates] = {0, -600, -2000};
int currWallState = 0;
int wallTarget = 0;

void nextWallState(){
	currWallState += 1;
	setControlMode(1);
	if(currWallState==numWallStates){
		currWallState=0;
	}
	wallTarget = wallStates[currWallState];
}

void liftControl(){
	double kp = 0.5;
	double average = (rightWallStake.get_position() + leftWallStake.get_position())/2;
	double error = wallTarget - average;
	double velocity = kp * error;
	leftWallStake.move(velocity);
	rightWallStake.move(velocity);
}

const int numAllianceStates = 3;
int allianceStates[numAllianceStates] = {0, -300, -3000};
int currAllianceState = 0;
int allianceTarget = 0;

void nextAllianceState(){
	currAllianceState += 1;
	setControlMode(2);
	if(currAllianceState==numAllianceStates){
		currAllianceState=0;
	}
	allianceTarget = allianceStates[currAllianceState];
}

void allianceControl(){
	double kp = 0.5;
	double average = (rightWallStake.get_position() + leftWallStake.get_position())/2;
	double error = allianceTarget - average;
	double velocity = kp * error;
	leftWallStake.move(velocity);
	rightWallStake.move(velocity);
}

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(12, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              22, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              21, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              800, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// pros::lcd::initialize();
	// pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(on_center_button);

	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

	clamp1.set_value(true);
	clamp2.set_value(false);

	pros::Task liftControlTask([]{
		while(true){
			if (controlMode == 1) {
            liftControl();
        	}
			pros::delay(10);
		}

	});


	pros::Task allianceControlTask([]{
		while(true){
			if (controlMode == 2) {
            allianceControl();
			}
			pros::delay(10);
		}

	});

	// print position to brain screen
    pros::Task screen_task([&]() {
	while (true) {
		// print robot location to the brain screen
		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		// delay to save resources
		pros::delay(20);
	}
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

// void intake_move_for(int seconds){
// 	//intake
// 	intake_bottom.move_velocity(-600);
// 	intake_top.move_velocity(600);
// 	seconds = seconds*1000;
// 	pros::delay(seconds); // 1000 miliseconds is 1 second
// 	intake_bottom.move_velocity(0);
// 	intake_top.move_velocity(0);
// }

// red ring and blue goal same
// blue ring and red goal same

void autonomous() {
	// set position to x:0, y:0, heading:0
    chassis.setPose(74.5, 9.75, 0); //was 12.25 add or minues 2.5
	chassis.turnToHeading(90, 2000);
	pros::delay(3000);
	chassis.turnToHeading(0, 2000);
	//move forward
	// chassis.moveToPoint(74.5, 24, 2000);
	// chassis.turnToHeading(0, 1000);
	// //move to mobile goal
	// chassis.turnToHeading(90, 1000);
	// chassis.moveToPoint(48, 24, 2000, {.forwards=false});
	// //chassis.turnToHeading(90, 1000, {}, false);
	// //clamp
	// pros::delay(1000);
	// clamp1.set_value(false);
	// clamp2.set_value(true);
	// pros::delay(1000);
	// //start intake
	// intake.move_velocity(-600);
	// //move to first ring
	// chassis.turnToHeading(0, 1000);
	// chassis.moveToPoint((48-5), (48+2.5), 2000);
	// chassis.turnToHeading(0, 1000);
	// //move to top of tile
	// chassis.turnToHeading(315, 1000);
	// chassis.moveToPoint((24-0), (72+0), 2000);
	// // score second ring
	// chassis.turnToHeading(270, 1000);
	// chassis.moveToPoint((12-2), (72+0), 2000);
	// chassis.turnToHeading(270, 1000);
	// pros::delay(1500);
	// // back up and turn
	// chassis.moveToPoint((24-0), (72+0), 2000, {.forwards=false});
	// chassis.turnToHeading(180, 1000);
	// //score third
	// chassis.moveToPoint((24-0), (48+0), 2000);
	// chassis.turnToHeading(180, 1000);
	// pros::delay(500);
	// //score fourth and fifth
	// chassis.moveToPoint((24-0), (12+4), 2000);
	// chassis.turnToHeading(180, 1000);
	// pros::delay(2000);
	// //drop in corner
	// chassis.moveToPoint((24-0), (24+0), 2000, {.forwards=false});
	// chassis.turnToHeading(45, 1000);
	// chassis.moveToPoint((12-0), (15+0), 2000, {.forwards=false});
	// chassis.turnToHeading(45, 1000);
	// pros::delay(1000);
	// clamp1.set_value(true);
	// clamp2.set_value(false);
	// //move to mobile goal and clamp
	// chassis.turnToHeading(45, 1000);
	// chassis.moveToPoint((24-0), (24+0), 2000);
	// chassis.turnToHeading(270, 1000);
	// chassis.moveToPoint((84-0), (24+0), 6000, {.forwards=false});
	// // clamp1.set_value(false);
	// clamp2.set_value(true);


}

// motor.move_absolute(100, 100); // at 100 (moves 100)
// motor.move_absolute(100, 100); // at 100 (moves 0)
// motor.set_zero_position(80);   // at 20
// motor.move_absolute(100, 100); // at 100 (moves 80)
//   // 
// std::uint32_t now = pros::millis();
// std::cout << "position = " << motor.get_position()  // prints "position = 100"
// std::cout << "raw position =" << motor.get_raw_position(&now); // prints "raw position = 180"

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
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

	while (true) {
        //pros::task_t(NULL);
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

		//intake
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move_velocity(-600);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.move_velocity(600);
		}
		else{
			intake.move_velocity(0);
		}

		// //wall stake
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
		// 	leftWallStake.move_velocity(-600);
		// 	rightWallStake.move_velocity(-600);
		// }
		// else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
		// 	leftWallStake.move_velocity(600);
		// 	rightWallStake.move_velocity(600);
		// }
		// else{
		// 	leftWallStake.move_velocity(0);
		// 	rightWallStake.move_velocity(0);
		// }

		//wall stake piston
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
		// 	wall_stake_piston.set_value(true);
		// }
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
		// 	wall_stake_piston.set_value(false);
		// }

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			setControlMode(1);
			nextWallState();
		}

		//aliance stake
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			setControlMode(2);
			nextAllianceState();
		}

		//clamp
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			clamp1.set_value(false);
			clamp2.set_value(true);
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			clamp1.set_value(true);
			clamp2.set_value(false);
		}

		pros::delay(25);
    }

	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	// pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	// while (true) {
	// 	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

	// 	// Arcade control scheme
	// 	int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
	// 	int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
	// 	left_mg.move(dir - turn);                      // Sets left motor voltage
	// 	right_mg.move(dir + turn);                     // Sets right motor voltage
	// 	pros::delay(20);                               // Run for 20 ms then update
	// }
}