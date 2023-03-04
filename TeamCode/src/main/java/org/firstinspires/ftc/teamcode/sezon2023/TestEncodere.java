package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Test Encodere", group="Linear Opmode")
@Config
public class TestEncodere extends LinearOpMode {

    DcMotorEx frontright,frontleft,backright,backleft;

    @Override
    public void runOpMode() throws InterruptedException {
        frontright = hardwareMap.get(DcMotorEx.class,"frontright");
        frontleft = hardwareMap.get(DcMotorEx.class,"frontleft");
        backright = hardwareMap.get(DcMotorEx.class,"backright");
        backleft = hardwareMap.get(DcMotorEx.class,"backleft");

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart ();

        while (opModeIsActive()) {
            telemetry.addData("frontright",frontright.getCurrentPosition());
            telemetry.addData("frontleft",frontleft.getCurrentPosition());
            telemetry.addData("backright",backright.getCurrentPosition());
            telemetry.addData("backleft",backleft.getCurrentPosition());
            telemetry.update();
        }
    }



    void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (Exception ex){

        }
    }

}
