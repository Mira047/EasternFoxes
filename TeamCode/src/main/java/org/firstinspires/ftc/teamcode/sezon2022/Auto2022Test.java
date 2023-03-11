package org.firstinspires.ftc.teamcode.sezon2022;

import android.hardware.camera2.params.BlackLevelPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.PathFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(name="Auto2022Test", group="Linear Opmode")
@Config
public class Auto2022Test extends LinearOpMode {

    public static double clawOpenIntake = 0.45;
    public static double extenderPower = 1;
    public static double anglePower = 1;
    public static int angleUp = -425;
    public static int angleMid = -650;
    public static int angleLow = -900;
    public static int extenderUp = -1600;
    public static int extenderMid = -1100;
    public static int extenderLow = -1100;
    public static double turretRotationUp = 120;
    public static double turretRotationMid = 120;
    public static double turretRotationLow = 120;

    CameraRecognition cameraRecognition;
    SampleMecanumDrive drive;
    Turret turret;
    DcMotor extender,angle;
    Servo claw,intake1,intake2;
    DistanceSensor distanceSensor;

    int freightInIntake=0;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class,"claw");
        intake1 = hardwareMap.get(Servo.class,"intake_servo1");
        intake2 = hardwareMap.get(Servo.class,"intake_servo2");
        extender = hardwareMap.get(DcMotor.class,"extender");
        angle = hardwareMap.get(DcMotor.class,"angle");
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(1);
        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setTargetPosition(0);
        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle.setPower(1);
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");
        intake1.setPosition(0.5);
        intake2.setPosition(0.5);

        turret = new Turret(hardwareMap,1);
        drive = new SampleMecanumDrive(hardwareMap);

        cameraRecognition =  new CameraRecognition(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);
        cameraRecognition.tresh = 140;

        //HardwareTesterInterpreter.initHWI(this,hardwareMap,telemetry,turret,drive);

        /*
        for(int i=0;i<trajectories.length;i++){
            Pose2d endPose = new Pose2d();
            if(i > 0){
                endPose = trajectories[i-1].end();
            }
        }
         */

        telemetry.addLine("Ready for start 2");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            try {
                main();
            }
            catch (Exception e){
                telemetry.addLine(e.toString());
                telemetry.update();
                Wait(5000);
            }
            break;
        }
    }

    private void Wait(int ms){
        try {
            Thread.sleep(ms);
        } catch (Exception ex){

        }
    }

    void waitForAnglePosition(int tresh){
        while (true && opModeIsActive() && !isStopRequested()){
            int p = angle.getCurrentPosition();
            int t = angle.getTargetPosition();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
    }

    void waitForExtenderPosition(int tresh){
        while (true && opModeIsActive() && !isStopRequested()){
            int p = extender.getCurrentPosition();
            int t = extender.getTargetPosition();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
    }

    void waitForTurretAngle(double tresh){
        while (true && opModeIsActive() && !isStopRequested()){
            double p = turret.getAngle();
            double t = turret.getTargetAngle();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
    }

    public static int target_freight = 3;
    public static double[][] extraPosX = {{28},{34},{36},{36},{-2},{-2}};
    public static double[][] extraPosY = {{-2},{-2},{-2},{-2},{-2},{-2}};
    public static double[][] extraPosRot = {{0},{0},{0},{0},{0},{0}};
    public static double[] turretRotIntake = {0,10,0,15,0,0};
    public static double turretPower = 0.5;
    public static double turretPower2 = 0.45;
    Trajectory[] trajectories = new Trajectory[extraPosX.length];

    public static int freightCase = 3;

    void onFreightCollect(){

    }

    void onFreightRelease(){

    }

    int currentFreight = 0;

    private void main(){

        currentFreight = 0;

        freightCase = cameraRecognition.getCase();

        Thread t = new Thread(){
            @Override
            public void run(){
                while (opModeIsActive() && !isStopRequested()){
                    double d = distanceSensor.getDistance(DistanceUnit.MM);
                    if(d>=60){
                        if(d>=90){
                            freightInIntake = -1;
                        } else {
                            freightInIntake = 0;
                        }
                    } else {
                        freightInIntake = 1;
                    }
                }
            }
        };
        t.start();


        telemetry.addLine("Case: " + Integer.toString(freightCase));
        telemetry.update();

        Teleop2022_v2.fromAuto = 1;

        double d=0;

        int ext = -1500;
        int ang = -440;

        double int1 = 0.45;
        double int2= 0.55;

        double turr = 117;

        if(freightCase == 1){
            ang = -875;
            ext = -1110;
            int1=0.25;
            int2=0.75;
            turr = 114;
        } else if(freightCase == 2){
            ang = -725;
            ext = -1200;
            turr = 116;
            int1=0.3;
            int2=0.7;
        } else if(freightCase == 3) {
            ext = -1500;
            ang = -440;
            int1=0.45;
            int2=0.55;
            turr = 113;
        }

            //HardwareTesterInterpreter.interpretScript("c2Vydm8gY2xhdyAwLjY1CndhaXQgMTAwCm1vdG9yIGV4dGVuZGVyIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgMSBwb3dlcgptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgYW5nbGUgMSBwb3dlcgptb3RvciBhbmdsZSAtNDUwIHBvc2l0aW9uCnR1cnJldCAwLjYgcG93ZXIKdHVycmV0IDExMCB3YWl0IDEKbW90b3IgYW5nbGUgd2FpdCBwb3NpdGlvbiA0MAptb3RvciBleHRlbmRlciAtMTUwMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDUwCnNlcnZvIGNsYXcgMC4zMgp3YWl0IDIwMAptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gNTAKdHVycmV0IDAuNiBwb3dlcgp0dXJyZXQgMCB3YWl0IDIwCnR1cnJldCAwLjYgcG93ZXIKbW90b3IgYW5nbGUgLTE0MDAgcG9zaXRpb24Kc2Vydm8gaW50YWtlX3NlcnZvMSAxCnNlcnZvIGludGFrZV9zZXJ2bzIgMApzZXJ2byBjbGF3IDAuNDUK","base64");
            claw.setPosition(0.65);
            Wait(100);
            angle.setTargetPosition(ang);
            turret.setPower(turretPower);
            turret.setRotationAsync(turr);
            waitForTurretAngle(1);
            waitForAnglePosition(40);
            extender.setTargetPosition(ext);
            waitForExtenderPosition(50);
        intake1.setPosition(int1);
        intake2.setPosition(int2);
        if(freightCase == 3) {
            claw.setPosition(0.3);
        } else {
            claw.setPosition(0.43);
        }
            while (freightInIntake != -1){
                Wait(0);
            }
            extender.setTargetPosition(0);
            waitForExtenderPosition(150);
            turret.setPower(turretPower);
            turret.setRotationAsync(turretRotIntake[0]);

            waitForTurretAngle(20 - Math.abs(turretRotIntake[0]));
            turret.setPower(turretPower);
            angle.setTargetPosition(-1400);

            intake1.setPosition(1);
            intake2.setPosition(0);
            claw.setPosition(0.45);


        currentFreight=1;
        for(int o=1;o<=target_freight;o++){
            //drive.followTrajectoryAsync(trajectories[currentFreight]);
            //angle.setTargetPosition(-700);
            if(isStopRequested())
            {
                break;
            }
            if(extraPosX[currentFreight].length == 1) {
                Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(extraPosX[currentFreight][0], extraPosY[currentFreight][0], Math.toRadians(extraPosRot[currentFreight][0])))
                        .build();
                drive.followTrajectoryAsync(traj);
            } else if(extraPosX[currentFreight].length == 2){
                Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(extraPosX[currentFreight][0], extraPosY[currentFreight][0], Math.toRadians(extraPosRot[currentFreight][0])))
                        .lineToLinearHeading(new Pose2d(extraPosX[currentFreight][1], extraPosY[currentFreight][1], Math.toRadians(extraPosRot[currentFreight][1])))
                        .build();
                drive.followTrajectoryAsync(traj);
            }
            telemetry.addLine("Waiting for collect " + Integer.toString(currentFreight));
            telemetry.update();
            int temp = 0;
            while(freightInIntake != 1 && !isStopRequested()) {
                if (drive.isBusy() == false) {
                    if (temp == 0) {
                        temp = 1;
                        turret.setRotationAsync(-9);
                    }
                    double a = turret.getAngle();
                    double d1 = Math.abs(20 - a);
                    if (d1 < 1) {
                        turret.setRotationAsync(-9);
                    }
                    double d2 = Math.abs(-9 - a);
                    if (d2 < 1) {
                        turret.setRotationAsync(20);
                    }
                }
                /*
                if(temp == 0) {
                    double pp = turret.getAngle();
                    double tt = turret.getTargetAngle();
                    if (pp >= tt - 20 && pp <= tt + 20) {
                        angle.setTargetPosition(-1400);
                        temp=1;
                    }
                }
                 */
                drive.update();
            }
            if(isStopRequested() || opModeIsActive() == false){
                break;
            }
            if(temp==1){
                Wait(150);
            }
            drive.breakFollowing();
            drive.setDrivePower(new Pose2d());
            drive.update();
            telemetry.addLine("Done collecting");
            telemetry.update();
            Wait(50);
            claw.setPosition(0.74);
            Wait(100);
            angle.setTargetPosition(-440);
            Wait(25);
            intake1.setPosition(0.5);
            intake2.setPosition(0.5);
            turret.setPower(turretPower2);
            turret.setRotationAsync(114+d);
            d+=0.9;

            Wait(150);

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(0,-2,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(traj);

            double r = Math.toDegrees(drive.getPoseEstimate().getHeading());
            if(r > 180){
                r = r - 360;
            }
            turret.setRotationAsync(114 + d - r);

            waitForAnglePosition(40);
            waitForTurretAngle(1);
            extender.setTargetPosition(-1500 + (int)(d * 30));
            waitForExtenderPosition(20);
            intake1.setPosition(0.1);
            intake2.setPosition(0);
            claw.setPosition(0.3);
            while (!isStopRequested() && freightInIntake != -1){
                Wait(0);
            }
            intake1.setPosition(0.5);
            intake2.setPosition(0.5);
            extender.setTargetPosition(0);
            waitForExtenderPosition(150);
            turret.setPower(turretPower);
            turret.setRotationAsync(turretRotIntake[currentFreight]);
            waitForTurretAngle(20 - Math.abs(turretRotIntake[currentFreight]));
            angle.setTargetPosition(-1400);
            intake1.setPosition(1);
            intake2.setPosition(0);
            claw.setPosition(0.46);
            currentFreight += 1;
            //HardwareTesterInterpreter.interpretScript(scriptHighFreight,"base64");
        }

        if(isStopRequested() || opModeIsActive() == false){
            return;
        }

        intake1.setPosition(0.5);
        intake2.setPosition(0.5);

        turret.setRotationAsync(0);

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(30,-2,Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj);

        Teleop2022_v2.startPose = drive.getPoseEstimate();
    }
}
