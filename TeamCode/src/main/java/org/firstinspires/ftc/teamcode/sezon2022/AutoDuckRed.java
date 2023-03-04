package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AutoDuckRed", group="Linear Opmode")
@Config
public class AutoDuckRed extends LinearOpMode {

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

    Servo duck;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        claw = hardwareMap.get(Servo.class,"claw");
        intake = hardwareMap.get(DcMotor.class,"duck");
        extender = hardwareMap.get(DcMotor.class,"extender");
        angle = hardwareMap.get(DcMotor.class,"angle");
        cap_y = hardwareMap.get(Servo.class,"cap_y");
        release = hardwareMap.get(Servo.class,"release");
        duck = hardwareMap.get(Servo.class,"duck_servo");
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(1);
        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setTargetPosition(0);
        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle.setPower(1);

        turret = new Turret(hardwareMap,1);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");
        intake.setPower(0);

        drive = new SampleMecanumDrive(hardwareMap);

        release.setPosition(0.4);


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

        double d=0;

        int ext = -1500;
        int ang = -750;


        double turr = 117;

        if(freightCase == 1){
            ext = -1550;
            ang = -700;
            turr = 57;
        } else if(freightCase == 2){
            ang = -1120;
            ext = -1300;
            turr = 57;
        } else if(freightCase == 3) {
            ext = -1550;
            ang = -700;
            turr = 57;
        }

        duck.setPosition(0.65);

        claw.setPosition(0.6);

        angle.setTargetPosition(-900);

        /*
        Wait(100);
        angle.setTargetPosition(ang);
        turret.setRotationAsync(turr);
        waitForTurretAngle(0.8);
        waitForAnglePosition(40);
        duck.setPosition(0.5);
        extender.setTargetPosition(ext);
        waitForExtenderPosition(30);
        if(freightCase == 3) {
            claw.setPosition(0.8);
            intake.setPower(0.4);
        } else {
            claw.setPosition(0.6);
            intake.setPower(0.4);
        }
        Wait(1500);
        extender.setTargetPosition(0);
        waitForExtenderPosition(50);
        angle.setTargetPosition(-1000);
        waitForAnglePosition(50);

        Wait(300);

        turret.setRotationAsync(0);
        waitForTurretAngle(0);

        if(isStopRequested() || opModeIsActive() == false){
            return;
        }

        intake.setPower(0);
        turret.setRotationAsync(0);


         */
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-31, 7, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj);

        duck.setPosition(0);

        Wait(7000);
        duck.setPosition(0.5);


        traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), 26, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj);


        Teleop2022_v2.startPose = drive.getPoseEstimate();
    }
}
