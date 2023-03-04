package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor {

    public Servo Down_s = null;
    public Servo Mid_s = null;
    public Servo Up_s = null;
    public Servo Pushq = null;
    public DcMotor Intake_motor = null;

    public Conveyor  (HardwareMap map) {
        Down_s = map.get (Servo.class, "intake_servo");
        Pushq = map.get (Servo.class, "ramp_3");
        Mid_s  = map.get (Servo.class, "ramp_1");
        Up_s   = map.get (Servo.class, "ramp_2");
        Intake_motor = map.get(DcMotor.class, "intake_motor");
    }

    public void move_down(double sgn) {
        Down_s.setPosition(0.5 - sgn);
    }
    public void move_up(double sgn) {
        Up_s.setPosition(0.5 + sgn);
    }
    public void move_mid(double sgn) {
        Mid_s.setPosition(0.5 - sgn);
    }
    public void push_shoot () {
        Pushq.setPosition(0.5);
    }

    public void push_retract()
    {
        Pushq.setPosition(0);
    }

    public void move_all() {
        move_down (-0.5);
        move_mid (-0.5);
        move_up (-0.5);
    }

    public void intake_start (double sgn) {
        Intake_motor.setPower (-1);
    }

    public void stop_all() {
        Down_s.setPosition (0.5);
        Up_s.setPosition (0.5);
        Mid_s.setPosition (0.5);
        Intake_motor.setPower (0);
    }

    public void stop_ramp_low(){
        Down_s.setPosition (0.5);
        Intake_motor.setPower (0);
        Mid_s.setPosition (0.5);
    }

    public void eject() {
        Intake_motor.setPower (1);
    }

    public void intake_stop() {
        Intake_motor.setPower (0);
    }

    public void stop_ramp_high() {
        Up_s.setPosition (0.5);
    }
}
