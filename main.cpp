#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup right_motors({9, 10, -8}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-13, -2, 3}, pros::MotorGearset::blue);
pros::Imu imu(7);

//vertical tracking wheel encoder
pros::Rotation vertical_encoder(6); //-2.875 inch from tracking center (7.5, 6.25), //-0.6 in
// vertical tracking wheel., front of the bot touched with offset of 4
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -1); //0.75, 2 came forward but clamp right of y,
//horizontal tracking wheel encoder
pros::adi::Encoder horizontal_encoder('E', 'F', true); //-3.75 in
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::OLD_275_HALF, 3); // began -4, -8 was far back on x, 4 left of y too far forward x, 1 worked, 3.63

pros::Motor intake_bottom(-20, pros::MotorGearset::blue);
pros::Motor intake_top(19, pros::MotorGearset::blue);
//pros::adi::Pneumatics clamp1("A");
//pros::adi::Pneumatics clamp2("H");
//pros::adi::Pneumatics clamp1 = pros::adi::Pneumatics('A'); // Create a pneumatic in port A
//pros::adi::digital clamp1('A'); // Create a pneumatic in port A
pros::adi::DigitalOut clamp1('A', false);
pros::adi::DigitalOut clamp2('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.25, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,//nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel,//nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              22, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
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
	// imu.reset();
	chassis.setPose(0, 0, 0);

	// print position to brain screen
    pros::Task screen_task([&]() {
	while (true) {
		// print robot location to the brain screen
		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		// print measurements from the rotation sensor
        pros::lcd::print(3, "Rotation Sensor: %i", vertical_encoder.get_position());
		// print measurements from the adi encoder
        // pros::lcd::print(4, "ADI Encoder: %i", horizontal_encoder.get_value());

		//pros::lcd::print(5, "IMU get heading: %f degrees\n", imu.get_heading());

		pros::lcd::print(5, "IMU: %f", imu.get_heading());
		// pros::lcd::print(6, "Horizontal: %f", horizontal_tracking_wheel.getDistanceTraveled());
		//pros::lcd::print(7, "Vertical: %f", vertical_tracking_wheel.getDistanceTraveled());
		// // delay to save resources
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
void competition_initialize() {
}

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

void intake_move_for(int seconds, int direction){
	//intake
	seconds = seconds*1000;
	if(direction == 1){
		intake_bottom.move_velocity(-600);
		intake_top.move_velocity(600);
		pros::delay(seconds); // 1000 miliseconds is 1 second
		intake_bottom.move_velocity(0);
		intake_top.move_velocity(0);
	}
	else if(direction == 0){
		intake_bottom.move_velocity(600);
		intake_top.move_velocity(-600);
		pros::delay(seconds); // 1000 miliseconds is 1 second
		intake_bottom.move_velocity(0);
		intake_top.move_velocity(0);
	}
}

void clamp(){
	clamp1.set_value(true);
	clamp2.set_value(false);
}

void unclamp(){
	clamp1.set_value(false);
	clamp2.set_value(true);
}

// red ring and blue goal same
// blue ring and red goal same

void autonomous() {
	// set position to x:0, y:0, heading:0
	// if async false waits until exit time before moving to next command dont need delay
	// use swing to heading for mobile goals
	//each field tile is 23.622 inches
	// 5.25+0.5+6.25 = 12
	
	// chassis.setPose(0, 0, 0);
	// // pros::delay(6000);
	// // chassis.turnToHeading(90, 1000);
	// chassis.moveToPoint(0, 24, 2000);
	// chassis.turnToHeading(0, 1000);
	// pros::delay(6000);
	// chassis.turnToHeading(90, 1000);
	// chassis.moveToPoint(24, 24, 2000);
	// chassis.turnToHeading(90, 1000);
	// pros::delay(6000);
	// chassis.turnToHeading(180, 1000);
	// chassis.moveToPoint(24, 0, 2000);
	// chassis.turnToHeading(180, 1000);
	// pros::delay(6000);
	// chassis.turnToHeading(270, 1000);
	// chassis.moveToPoint(0, 0, 2000);
	// chassis.turnToHeading(0, 1000);




	chassis.setPose(72, 12, 0); //11.9375, was 10.75
	pros::delay(1000);
	// unclamp
	clamp1.set_value(false);
	clamp2.set_value(true);

	//clamp
	chassis.moveToPoint(72, (24-1), 2000);
	// chassis.moveToPose(72, 24, 0, 2000);
	chassis.turnToHeading(0, 1000);
	//pros::delay(5000);
	// chassis.turnToHeading(90,1000);
	chassis.turnToHeading(90, 1000);
	// chassis.moveToPose(59, 24, 90, 3000, {.minSpeed = 100}, false);
	//pros::delay(2000);
	chassis.moveToPoint(60, (24-1), 10000, {.forwards=false});
	//pros::delay(1000);
	//chassis.turnToHeading(270, 1000);
	chassis.moveToPoint(50, (24-1), 1000, {.forwards=false}, false);
	clamp1.set_value(true);
	clamp2.set_value(false);
	//score ring 1
	//chassis.moveToPoint((48+6), 24, 1000);
	chassis.turnToHeading(0, 1000);
	intake_bottom.move_velocity(-600);
	intake_top.move_velocity(600);
	chassis.moveToPoint((48+2), (48-4), 2000);
	chassis.turnToHeading(0, 1000);
	//move to top of tile
	chassis.turnToHeading(315, 1000);
	chassis.moveToPoint(33, 74, 2000);
	//score second ring
	chassis.turnToHeading(270, 1000);
	chassis.moveToPoint(22, 75, 1000);
	chassis.turnToHeading(270, 1000);
	//back up and face rings 3, 4 and 5
	chassis.moveToPoint(24, 75, 2000, {.forwards=false});
	chassis.turnToHeading(270, 1000);
	chassis.turnToHeading(182, 1000);
	//score rings 3, 4, and 5
	chassis.moveToPoint((24-2), (24-6), 2500, {.maxSpeed=90});
	pros::delay(3000);
	//back into corner and unclamp
	chassis.moveToPoint(26, (20+2), 1500, {.maxSpeed=90}, false);//originally 20
	chassis.turnToHeading(45, 1000);
	chassis.moveToPoint(10, 10, 1500, {.forwards = false}, false);
	clamp1.set_value(false);
	clamp2.set_value(true);
	//back out of corner and move in line to mobile goal
	chassis.turnToHeading(45, 1000);
	intake_bottom.move_velocity(0);
	intake_top.move_velocity(0);
	intake_top.move_velocity(-600);
	pros::delay(800);
	intake_top.move_velocity(0);
	chassis.moveToPoint(28, (20-2), 1500);//originally 20
	chassis.turnToHeading(270, 1000);
	//clamp second mobile goal
	chassis.moveToPoint((72+6), (24-10), 5000,{.forwards = false});// was 24-9
	chassis.turnToHeading(270, 1000);
	//pros::delay(500);
	chassis.moveToPoint((96-6), (24-12), 1000,{.forwards = false});//was 24-11
	chassis.turnToHeading(270, 1000);
	clamp1.set_value(true);
	clamp2.set_value(false);
	//get first ring
	chassis.moveToPoint((96-2), (24-12), 1000,{.forwards = false});
	intake_bottom.move_velocity(-600);
	intake_top.move_velocity(600);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPoint((96-2), 36, 2000);
	chassis.turnToHeading(0, 1000);
	//move to top of tile
	chassis.turnToHeading(50, 1000);
	chassis.moveToPoint(112, 57, 2000);
	//score ring 2
	chassis.turnToHeading(90, 1000);
	chassis.moveToPoint(121, 59, 2000, {}, false);
	chassis.turnToHeading(90, 1000);
	//back up
	chassis.moveToPoint(116, 59, 2000, {.forwards=false});
	chassis.turnToHeading(90, 1000);
	chassis.turnToHeading(178, 1000);
	//score rings 3, 4, and 5
	chassis.moveToPoint((120-4), (24-8), 2500, {.maxSpeed=90});
	pros::delay(2000);
	chassis.moveToPoint((120-4), (24-18), 2500, {.maxSpeed=90});
	pros::delay(1500);
	//back into corner and unclamp
	chassis.moveToPoint((120-8), (24-10), 1500, {.forwards = false});
	chassis.turnToHeading(315, 1000);
	chassis.moveToPoint(130, -4, 2000, {.forwards = false}, false);
	clamp1.set_value(false);
	clamp2.set_value(true);
	//back out of corner and line up to other side of field
	chassis.turnToHeading(315, 1000);
	intake_bottom.move_velocity(0);
	intake_top.move_velocity(0);
	intake_top.move_velocity(-600);
	pros::delay(800);
	intake_top.move_velocity(0);
	chassis.moveToPoint((120-8), (24-6), 2000);
	chassis.turnToHeading(0, 1000);
	//move to other side of field
	intake_bottom.move_velocity(-600);
	intake_top.move_velocity(600);
	chassis.moveToPoint((120-8), (108-26), 7500, {.minSpeed=80});
	chassis.turnToHeading(270, 1000);
	
	//get behind mobile goal
	chassis.moveToPoint((84-7), (108-24), 4000);//was 84-5
	//chassis.turnToHeading(270, 1000);
	intake_bottom.move_velocity(0);
	intake_top.move_velocity(0);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPoint((84-3), (120+4), 1000); //was 84-3
	chassis.turnToHeading(85, 1000);
	//push mobile in
	chassis.moveToPoint((120+2), (120+6), 1400);
	//back up and face second mobile goal
	chassis.turnToHeading(90, 1000);
	chassis.moveToPoint((120-10), (120-6), 4000, {.forwards=false});
	chassis.turnToHeading(270, 1000);
	//push second mobile goal in
	chassis.moveToPoint(48, 120-4, 2500, {.minSpeed=90});
	chassis.moveToPoint(8, 120+13, 10000, {.minSpeed=95});


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
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

	while (true) {
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			chassis.setPose(72, 12, 0); //11.9375, was 10.75
			pros::delay(1000);
			// unclamp
			clamp1.set_value(false);
			clamp2.set_value(true);

			//clamp
			chassis.moveToPoint(72, (24-1), 2000);
			// chassis.moveToPose(72, 24, 0, 2000);
			chassis.turnToHeading(0, 1000);
			//pros::delay(5000);
			// chassis.turnToHeading(90,1000);
			chassis.turnToHeading(90, 1000);
			// chassis.moveToPose(59, 24, 90, 3000, {.minSpeed = 100}, false);
			//pros::delay(2000);
			chassis.moveToPoint(60, (24-1), 10000, {.forwards=false});
			//pros::delay(1000);
			//chassis.turnToHeading(270, 1000);
			chassis.moveToPoint(50, (24-1), 1000, {.forwards=false}, false);
			clamp1.set_value(true);
			clamp2.set_value(false);
			//score ring 1
			//chassis.moveToPoint((48+6), 24, 1000);
			chassis.turnToHeading(0, 1000);
			intake_bottom.move_velocity(-600);
			intake_top.move_velocity(600);
			chassis.moveToPoint((48+2), (48-4), 2000);
			chassis.turnToHeading(0, 1000);
			//move to top of tile
			chassis.turnToHeading(315, 1000);
			chassis.moveToPoint(33, 74, 2000);
			//score second ring
			chassis.turnToHeading(270, 1000);
			chassis.moveToPoint(22, 75, 1000);
			chassis.turnToHeading(270, 1000);
			//back up and face rings 3, 4 and 5
			chassis.moveToPoint(24, 75, 2000, {.forwards=false});
			chassis.turnToHeading(270, 1000);
			chassis.turnToHeading(182, 1000);
			//score rings 3, 4, and 5
			chassis.moveToPoint((24-2), (24-6), 2500, {.maxSpeed=90});
			pros::delay(3000);
			//back into corner and unclamp
			chassis.moveToPoint(26, (20+2), 1500, {.maxSpeed=90}, false);//originally 20
			chassis.turnToHeading(45, 1000);
			chassis.moveToPoint(10, 10, 1500, {.forwards = false}, false);
			clamp1.set_value(false);
			clamp2.set_value(true);
			//back out of corner and move in line to mobile goal
			chassis.turnToHeading(45, 1000);
			intake_bottom.move_velocity(0);
			intake_top.move_velocity(0);
			intake_top.move_velocity(-600);
			pros::delay(800);
			intake_top.move_velocity(0);
			chassis.moveToPoint(28, (20-2), 1500);//originally 20
			chassis.turnToHeading(270, 1000);
			//clamp second mobile goal
			chassis.moveToPoint((72+6), (24-10), 5000,{.forwards = false});// was 24-9
			chassis.turnToHeading(270, 1000);
			//pros::delay(500);
			chassis.moveToPoint((96-6), (24-12), 1000,{.forwards = false});//was 24-11
			chassis.turnToHeading(270, 1000);
			clamp1.set_value(true);
			clamp2.set_value(false);
			//get first ring
			chassis.moveToPoint((96-2), (24-12), 1000,{.forwards = false});
			intake_bottom.move_velocity(-600);
			intake_top.move_velocity(600);
			chassis.turnToHeading(0, 1000);
			chassis.moveToPoint((96-2), 36, 2000);
			chassis.turnToHeading(0, 1000);
			//move to top of tile
			chassis.turnToHeading(50, 1000);
			chassis.moveToPoint(112, 57, 2000);
			//score ring 2
			chassis.turnToHeading(90, 1000);
			chassis.moveToPoint(121, 59, 2000, {}, false);
			chassis.turnToHeading(90, 1000);
			//back up
			chassis.moveToPoint(116, 59, 2000, {.forwards=false});
			chassis.turnToHeading(90, 1000);
			chassis.turnToHeading(178, 1000);
			//score rings 3, 4, and 5
			chassis.moveToPoint((120-4), (24-8), 2500, {.maxSpeed=90});
			pros::delay(2000);
			chassis.moveToPoint((120-4), (24-18), 2500, {.maxSpeed=90});
			pros::delay(1500);
			//back into corner and unclamp
			chassis.moveToPoint((120-8), (24-10), 1500, {.forwards = false});
			chassis.turnToHeading(315, 1000);
			chassis.moveToPoint(130, -4, 2000, {.forwards = false}, false);
			clamp1.set_value(false);
			clamp2.set_value(true);
			//back out of corner and line up to other side of field
			chassis.turnToHeading(315, 1000);
			intake_bottom.move_velocity(0);
			intake_top.move_velocity(0);
			intake_top.move_velocity(-600);
			pros::delay(800);
			intake_top.move_velocity(0);
			chassis.moveToPoint((120-8), (24-6), 2000);
			chassis.turnToHeading(0, 1000);
			//move to other side of field
			intake_bottom.move_velocity(-600);
			intake_top.move_velocity(600);
			chassis.moveToPoint((120-8), (108-26), 7500, {.minSpeed=80});
			chassis.turnToHeading(270, 1000);
			
			//get behind mobile goal
			chassis.moveToPoint((84-7), (108-24), 4000);//was 84-5
			//chassis.turnToHeading(270, 1000);
			intake_bottom.move_velocity(0);
			intake_top.move_velocity(0);
			chassis.turnToHeading(0, 1000);
			chassis.moveToPoint((84-3), (120+4), 1000); //was 84-3
			chassis.turnToHeading(85, 1000);
			//push mobile in
			chassis.moveToPoint((120+2), (120+6), 1800);
			//back up and face second mobile goal
			chassis.turnToHeading(90, 1000);
			chassis.moveToPoint((120-10), (120-4), 4000, {.forwards=false});
			chassis.turnToHeading(270, 1000);
			//push second mobile goal in
			chassis.moveToPoint(48, 120-4, 2500, {.minSpeed=90});
			chassis.moveToPoint(8, 120+13, 10000, {.minSpeed=95});
		}
        //pros::task_t(NULL);
        // get left y and right y positions
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // // move the robot
        // chassis.tank(leftY, rightY);

		// //intake
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
		// 	intake_bottom.move_velocity(-600);
		// 	intake_top.move_velocity(600);
		// }
		// else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		// 	intake_bottom.move_velocity(600);
		// 	intake_top.move_velocity(-600);
		// }
		// else{
		// 	intake_bottom.move_velocity(0);
		// 	intake_top.move_velocity(0);
		// }

		// //clamp
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
		// 	clamp1.set_value(false);
		// 	clamp2.set_value(true);
		// }
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
		// 	clamp1.set_value(true);
		// 	clamp2.set_value(false);
		// }

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