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

    pros::Motor_Group l_base({lb_base,lt_base,lf_base});
    pros::Motor_Group r_base({rb_base,rt_base,rf_base});
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

void move_pid(pros::Motor_Group* l_motors, pros::Motor_Group* r_motors, double t_distance_l, double t_distance_r){
    const double kp = 1.0;
    const double ki = 0.0;
    const double kd = 15.0;
    double pos_l = l_motors->get_positions().back();
    t_distance_l += pos_l;
    bool at_target = false;
    double error_l = t_distance_l - pos_l;
    double prev_error_l = t_distance_l - pos_l;
    double integral_l = 0.0;
    double p_term_l;
    double i_term_l;
    double d_term_l;
    double out_l;

    double pos_r = r_motors->get_positions().back(); 
    t_distance_r += pos_r;
    double error_r = t_distance_r - pos_r;
    double prev_error_r = t_distance_r - pos_r;
    double integral_r = 0.0;
    double p_term_r;
    double i_term_r;
    double d_term_r;
    double out_r;
    while (!at_target){
        pos_l = l_motors->get_positions().back();
        pos_r = r_motors->get_positions().back(); 
        prev_error_l = error_l;
        prev_error_r = error_r;
        error_l = t_distance_l - pos_l;
        error_r = t_distance_r - pos_r; 
        if (error_l <= 1.0 && error_r<=1.0){
            double v_l = l_motors->get_actual_velocities().back();
            double v_r = r_motors->get_actual_velocities().back();
            if(v_l<1.0 && v_r<1.0){
                at_target =true;
                break;
            }
        }
        p_term_l = kp * error_l;
        integral_l += error_l;
        i_term_l = ki * integral_l;
        d_term_l = kd * (error_l - prev_error_l);
        out_l = p_term_l+i_term_l+d_term_l;

        p_term_r = kp * error_r;
        integral_r += error_r;
        i_term_r = ki * integral_r;
        d_term_r = kd * (error_r - prev_error_r);
        out_r = p_term_r+i_term_r+d_term_r;
        printf("outl=%f, outr=%f | ", out_l, out_r);
        int out_l_int = (out_l>127.0)?127:(out_l<-127.0)?-127:(int) out_l;
        int out_r_int = (out_r>127.0)?127:(out_r<-127.0)?-127:(int) out_r;
        printf("outlint=%d, outrint=%d \n", out_l_int, out_r_int);
        l_motors->move(out_l_int);
        r_motors->move(out_r_int);
        pros::delay(5);
    }
}

void move_line(double dist){
    //base motors
  pros::Motor lf_base(lf_port);
  pros::Motor lt_base(lt_port);
  pros::Motor lb_base(lb_port);
  pros::Motor rf_base(rf_port);
  pros::Motor rt_base(rt_port);
  pros::Motor rb_base(rb_port);
    
    pros::Motor_Group l_base({lb_base,lt_base,lf_base});
    pros::Motor_Group r_base({rb_base,rt_base,rf_base});

    const double pi = 3.1415926535897932;
    double rot = dist/(2.75*25.48*pi)*360;
    move_pid(&l_base, &r_base, rot, rot);
    pros::delay(10);
}

void move_turn(double theta){
    //base motors
  pros::Motor lf_base(lf_port);
  pros::Motor lt_base(lt_port);
  pros::Motor lb_base(lb_port);
  pros::Motor rf_base(rf_port);
  pros::Motor rt_base(rt_port);
  pros::Motor rb_base(rb_port);
    
    pros::Motor_Group l_base({lb_base,lt_base,lf_base});
    pros::Motor_Group r_base({rb_base,rt_base,rf_base});

    const double pi = 3.1415926535897932;
    const double base_width = 310.0;
    double l_rot = (base_width*(theta*2.0*pi/360.0)/2.0)/(2.75*25.48*pi)*360.0;
    
    printf("L_ROT = %f\n", l_rot);
    double r_rot = -l_rot;
    printf("R_ROT = %f\n", r_rot);
    move_pid(&l_base, &r_base, l_rot, r_rot);
    printf("fuck\n");

    pros::delay(10);
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
    
    pros::Motor_Group l_base({lb_base,lt_base,lf_base});
    pros::Motor_Group r_base({rb_base,rt_base,rf_base});

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
    
    //Auton test code
    /*
    double dist = 600.0;
    const double pi = 3.1415926535897932;
    double l_rot = dist/(2.75*25.48*pi)*360;
    double r_rot = dist/(2.75*25.48*pi)*360;*/
    move_pid(&l_base, &r_base, -500, -500);

    move_line(600.0);
    printf("+600\n");
    pros::delay(1000);
    move_line(-600.0);
    printf("-600\n");
    pros::delay(1000);
    move_turn(30);
    
    printf("30\n");
    
    pros::delay(1000);
  while(true){//base control
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