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
    pros::Motor lc(lc_port, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_port, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
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
    int flipper_state = 0;
    double flipper_target = 270, flipper_power = 100, flipper_kp = 1, intake_power = 120, flipper_error = 0;

    //cata motors
    pros::Motor lc(lc_port);
    pros::Motor rc(rc_port);

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
        if(master.get_digital_new_press(DIGITAL_X)){
            //flipper_state = 1;
            flipper_target = 270;
            //fs.move(flipper_error * flipper_kp);
            fr.move(0);
        }

        else if(master.get_digital_new_press(DIGITAL_B)){
            //flipper_state = 2;
            flipper_target = 300;
        }

        //master.print(1, 1, "Flipper State: %d", flipper_state);
        //master.print(0, 0, "Flipper Rot: %d", flipperrot.get_position());
        // master.print(0, 0, "Flipper Error: %d", flipper_error);
        // master.print(0, 0, "Flipper Target: %d", flipper_target);
        master.print(0, 0, "rot:", flipperrot.get_position());
        

        /*
        if(master.get_digital(DIGITAL_R1)){
            fs.move(100);
            fr.move(100);
        }

        else if(master.get_digital(DIGITAL_R2)){
            fs.move(-100);
            fr.move(-100);
        }

        else{
            fs.move(0);
            fr.move(0);
        }
        */
        
        //cata control
        lc.move(-70*master.get_digital(DIGITAL_L1));
        rc.move(-70*master.get_digital(DIGITAL_L1));

        pros::delay(5);
	}
}
