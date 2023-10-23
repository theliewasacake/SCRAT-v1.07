#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#define flipper_targetUp 240
#define flipper_targetDown 320
#define flipper_kp 2
#define flipper_kd 250
#define flipper_ki 0.001
#define RpmToRad 3.141 / 60
#define RadToRpm 60 / 3.141
#define FlipperMotorMaxRPM 100
#define decelConstant 0.0002

#define cata_kp 4
#define cata_kd 0
#define Catadelay 400
#define allowedError 2
#define cata_target 214
#define cata_power 50

void initialize() {

    //controller
    pros::Controller master(CONTROLLER_MASTER);

	//base
    pros::Motor lf_base(lf_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lt_base(lt_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lb_base(lb_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rf_base(rf_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rt_base(rt_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rb_base(rb_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

    //flipper
    pros::Motor fs(fs_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor fr(fr_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation flipperrot(flipperrot_port);
    
    //cata
    pros::Motor lc(lc_port, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    // lc.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // rc.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Rotation catarot(catarot_port);

    //side rollers
    pros::Motor lr (lr_port, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor rr (lr_port, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
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

    bool flipperPosUp = true; //false means down, true means up
    int flipper_target;
    float flipper_error;
    float prev_flipper_error;
    float flipper_d;
    float total_flipper_error;

    //variables for 2131 transmission inverse functions
    float TargetOmegaI = 0.0;
    float TargetOmegaA = 0.0;
    float TargetOmegaFS;
    float TargetOmegaFR;

    //cata motors
    pros::Motor lc(lc_port);
    pros::Motor rc(rc_port);
    pros::Rotation catarot(catarot_port);

    int cata_error;
    int prev_cata_error;
    int cata_d;
    uint32_t timestamp;
    int correctingPow;

    //side rollers motor
    pros::Motor lr(lr_port);
    pros::Motor rr(rr_port);

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

        //update target speeds for I and update target position for flipper
        if(master.get_digital_new_press(DIGITAL_X))
            flipper_target = flipper_targetUp; //move to up position
        else if(master.get_digital_new_press(DIGITAL_B))
            flipper_target = flipper_targetDown; //move to down position
        if(master.get_digital_new_press(DIGITAL_DOWN))
            TargetOmegaI = -5.236; //roller outtake
        else if(master.get_digital_new_press(DIGITAL_UP))
            TargetOmegaI = 5.236; //roller intake

        

        //PID loop to get the arm to the target position
        //calculates TargetOmegaA, and ActualOmegaFS will be changed according to the PID loop in order to reach the target encoder value given by flipper_target
        flipper_error = flipperrot.get_position() / 100 - flipper_target;

        //finding target rotation rate of the arm in rad/s
        TargetOmegaA = flipper_error * decelConstant * RadToRpm;
        printf("flippereror: %f \n", flipper_error);

        //calculate required input speeds of motors to get desired transmission output config in rad/s
        TargetOmegaFS = (TargetOmegaA + TargetOmegaI) * (float) 5.0; 
        TargetOmegaFR = TargetOmegaI * (float) 5.0;

        //convert TargetOmegaFS and TargetOmegaFR from rad/s to rpm
        TargetOmegaFR = TargetOmegaFR * RadToRpm;
        TargetOmegaFS = TargetOmegaFS * RadToRpm;
        printf("tOmegaA: %f \n", TargetOmegaA);
        printf("tOmegaFS: %f \n", TargetOmegaFS);
        printf("tOmegaFR: %f \n", TargetOmegaFR);

        fs.move_velocity(TargetOmegaFS);
        fr.move_velocity(TargetOmegaFR);

        
        
        
        //updating values of these global variables
        cata_error = cata_target - catarot.get_position()/100;
        cata_d = cata_error - prev_cata_error;
        correctingPow = cata_error * cata_kp + cata_d * cata_kd + cata_power;
        
        // latest cata control
        if(master.get_digital(DIGITAL_L2)){
            lc.move(30);
            rc.move(30);
            timestamp = pros::millis();
        }
        
        else if(pros::millis() - timestamp > Catadelay){
            //delay is time taken for catapult arm to fully fire
            //timestamp records the moment from which the catapult began spinning and the slip gear slips
            //after some delay, the catapult has fired and the slip gear can then begin to rewind
            if(catarot.get_position() > cata_target //if we are still undershooting
            && abs(cata_error) > allowedError){ //magnitude of catapult error is greater than allowed error
                lc.move(correctingPow);
                rc.move(correctingPow);
            }

            else{
                lc.move(0);
                rc.move(0);
            }
        }
        
        //cata debugging
        // printf("Position: %i \n", catarot.get_position()/100);
        // printf("Error: %i \n", cata_error);
        // printf("CorrectingPow: %i \n", correctingPow);
        // printf("Current: %i \n", lc.get_current_draw());

        //side rollers control
        lr.move(100 * (master.get_digital(DIGITAL_R2) - master.get_digital(DIGITAL_R1)));
        rr.move(100 * (master.get_digital(DIGITAL_R2) - master.get_digital(DIGITAL_R1)));


        pros::delay(5);
	}
}