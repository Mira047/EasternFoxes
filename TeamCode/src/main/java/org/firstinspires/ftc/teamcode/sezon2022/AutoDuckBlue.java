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

@Autonomous(name="AutoDuckBlue", group="Linear Opmode")
@Config
public class AutoDuckBlue extends LinearOpMode {

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
    Servo claw;
    Servo release;
    DcMotor intake;
    DistanceSensor distanceSensor;

    int freightInIntake=0;

    DcMotor duck;
    Servo intake1,intake2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        claw = hardwareMap.get(Servo.class,"claw");
        duck = hardwareMap.get(DcMotor.class,"duck");
        extender = hardwareMap.get(DcMotor.class,"extender");
        angle = hardwareMap.get(DcMotor.class,"angle");
        intake1 = hardwareMap.get(Servo.class,"intake1");
        intake2 = hardwareMap.get(Servo.class,"intake2");
        //cap_y = hardwareMap.get(Servo.class,"cap_y");
        //duck = hardwareMap.get(Servo.class,"duck_servo");
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(1);
        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setTargetPosition(0);
        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle.setPower(1);

        turret = new Turret(hardwareMap,1);

        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

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
    public static double[][] extraPosX = {{28},{34},{36},{36},{38},{-2},{-2}};
    public static double[][] extraPosY = {{-2},{-2},{-2},{-2},{-2},{-2},{-2}};
    public static double[][] extraPosRot = {{0},{0},{0},{0},{0},{0},{0}};
    public static double[] turretRotIntake = {0,10,0,15,0,0,0};
    public static double turretPower = 0.42;
    public static double turretPower2 = 0.42;
    Trajectory[] trajectories = new Trajectory[extraPosX.length];

    public static int freightCase = 3;

    Servo cap_y;

    void onFreightCollect(){

    }

    void onFreightRelease(){

    }

    int currentFreight = 0;

    private void main(){


        currentFreight = 0;

        freightCase = cameraRecognition.getCase();


        telemetry.addLine("Case: " + Integer.toString(freightCase));
        telemetry.update();

        Teleop2022_v2.fromAuto = 1;
        Teleop2022Blue.fromAuto = 1;

        //duck.setPosition(0.65);

        claw.setPosition(1);

        angle.setTargetPosition(-800);



        int ang = -800;
        double turr = 115;
        int ext = -1350;
        Wait(2000);
        angle.setTargetPosition(ang);
        turret.setRotationAsync(turr);
        waitForTurretAngle(0.8);
        waitForAnglePosition(40);
        extender.setTargetPosition(ext);
        waitForExtenderPosition(70);
        claw.setPosition(0.5);
        intake1.setPosition(0.7);
        intake2.setPosition(0.3);
        Wait(600);
        claw.setPosition(0.5);
        intake1.setPosition(0.5);
        intake2.setPosition(0.5);
        extender.setTargetPosition(0);
        waitForExtenderPosition(50);
        angle.setTargetPosition(-2000);
        waitForAnglePosition(50);


        if(isStopRequested() || opModeIsActive() == false){
            return;
        }



        /*

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-26, -4, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj);

        duck.setPower(0.3);
        Wait(5000);
        duck.setPower(-0.3);
        Wait(5000);
        duck.setPower(0);


        traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), -8, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj);
*/

        Teleop2022_v2.startPose = drive.getPoseEstimate();
    }
}
