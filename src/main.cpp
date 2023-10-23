#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#define flipper_targetUp 240 //degrees
#define flipper_targetDown 320 //degrees
#define flipper_kp 2
#define flipper_kd 250
#define flipper_ki 0.001
#define RpmToRad 3.141 / 60
#define RadToRpm 60 / 3.141

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
	pros::Motor fr(fr_port, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
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


    bool IntakeTargetPosUp = true;
    float flipper_error;
    float prev_flipper_error;
    float flipper_d;
    float total_flipper_error;

    //variables for 2131 transmission inverse functions
    float TargetRollerRPM;
    float TargetArmRPM;
    float OmegaBP; //BP is the motor on the arm because the transmission is upside down
    float OmegaAP; //AP is the motor on the base because the transmission is upside down
    float TargetOmegaA;
    float prevflipper_error;

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
            IntakeTargetPosUp = true; //move to up position
        else if(master.get_digital_new_press(DIGITAL_B))
            IntakeTargetPosUp = false; //move to down position

        if(master.get_digital_new_press(DIGITAL_DOWN))
            TargetRollerRPM = -500;//roller outtake
        else if(master.get_digital_new_press(DIGITAL_UP))
            TargetRollerRPM = 500; //roller intake

        int currentPos = flipperrot.get_position() / 100;
        //PID loop to get the arm to the target position
        //calculates TargetOmegaA, and ActualOmegaFS will be changed according to the PID loop in order to reach the target encoder value given by flipper_target
        
        if(IntakeTargetPosUp)
            flipper_error = currentPos - flipper_targetUp;
        else
            flipper_error = currentPos - flipper_targetDown;

        prev_flipper_error = flipper_error;

        TargetOmegaA = flipper_error * flipper_kp + prev_flipper_error * flipper_kd;

        OmegaBP = -TargetRollerRPM / 5;
        OmegaAP = (TargetOmegaA + 0.2 * OmegaBP) * 5;

        fs.move_velocity(OmegaAP);
        fr.move_velocity(OmegaBP);
        
        printf("TargetRollerRPM: %f \n", TargetRollerRPM);
        printf("TargetOmegaA: %f \n", TargetOmegaA);
        printf("OmegaAP: %f \n", OmegaAP);
        printf("OmegaBP: %f \n", OmegaBP);

        
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