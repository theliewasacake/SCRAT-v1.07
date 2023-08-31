#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

void initialize() {

    //controller
    pros::Controller master(CONTROLLER_MASTER);

	//base
    pros::Motor lf_base(lf_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lt_base(lt_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lb_base(lb_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rf_base(rf_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rt_base(rt_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rb_base(rb_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

    //flipper
    pros::Motor fs(fs_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor fr(fr_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation flipperrot(flipperrot_port);
    
    //cata
    pros::Motor lc(lc_port, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_port, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot(catarot_port);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	//controller
    pros::Controller master(CONTROLLER_MASTER);

	//base motors
	pros::Motor lf_base(lf_port);
	pros::Motor lt_base(lt_port);
	pros::Motor lb_base(lb_port);
	pros::Motor rf_base(rf_port);
	pros::Motor rt_base(rt_port);
	pros::Motor rb_base(rb_port);

	//drive mode control
	bool tankdrive = true;

    //flipper motors
    pros::Motor fs(fs_port);
    pros::Motor fr(fr_port);
    pros::Rotation flipperrot(flipperrot_port);
    int flipper_target = 270;
    int flipper_power = 100, intake_power = 120;
    int flipper_kp = 2, flipper_error = 0;
    int flipper_kd = 250, prev_flipper_error = 0, flipper_d = 0;
    int flipper_ki = 0.001, total_flipper_error = 0;

    //cata motors
    pros::Motor lc(lc_port);
    pros::Motor rc(rc_port);
    pros::Rotation catarot(catarot_port);
    int cata_target = 210, cata_power = 50;
    int cata_kp = 2, cata_error = 0;
    int cata_kd = 2000, prev_cata_error = 0, cata_d = 0;

	while(true){

        //base control
        double left, right;
        if(master.get_digital_new_press(DIGITAL_Y)) tankdrive = !tankdrive;
        if(tankdrive) {
            left = master.get_analog(ANALOG_LEFT_Y);
            right = master.get_analog(ANALOG_RIGHT_Y);
        } 
                
        else {
            double power =  master.get_analog(ANALOG_LEFT_Y);
            double turn = master.get_analog(ANALOG_RIGHT_X);
            left = power + turn;
            right = power - turn;
        }

        lf_base.move(left);
        lt_base.move(left);
        lb_base.move(left);
        rf_base.move(right);
        rt_base.move(right);
        rb_base.move(right);

        //flipper control
        flipper_error = flipperrot.get_position()/ 100 - flipper_target;
        flipper_d = flipper_error - prev_flipper_error;
        total_flipper_error += flipper_error;
        prev_flipper_error = flipper_error;

        fs.move(flipper_error * flipper_kp + total_flipper_error * flipper_ki + intake_power * (master.get_digital(DIGITAL_DOWN) - master.get_digital(DIGITAL_UP))); 
        fr.move(-flipper_error * flipper_kp + total_flipper_error * flipper_ki + intake_power * (master.get_digital(DIGITAL_DOWN) - master.get_digital(DIGITAL_UP)));

        if(master.get_digital_new_press(DIGITAL_X)){
            //up position
            flipper_target = 270;
        }

        else if(master.get_digital_new_press(DIGITAL_B)){
            // down position
            flipper_target = 190;
            fr.move(10);
        }

        //printf("Target: %i \n", flipper_target);
        //printf("Error: %i \n", flipper_error);
        //printf("Power: %i \n", flipper_error * flipper_kp + total_flipper_error * flipper_ki + intake_power * (master.get_digital(DIGITAL_DOWN) - master.get_digital(DIGITAL_UP)));
        
        //cata control

        /*
        lc.move(40*master.get_digital(DIGITAL_L1));
        rc.move(40*master.get_digital(DIGITAL_L1));

        cata_error = cata_target - catarot.get_position()/100;
        cata_d = cata_error - prev_cata_error;
        */

        // lc.move(cata_error * cata_kp + cata_d * cata_kd + cata_power);
        // rc.move(cata_error * cata_kp + cata_d * cata_kd + cata_power);

        /*
        printf("Position: %i \n", catarot.get_position()/100);
        printf("Error: %i \n", cata_error);
        */


        pros::delay(5);
	}
}