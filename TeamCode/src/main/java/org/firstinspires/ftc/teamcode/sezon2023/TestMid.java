package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Test Mid", group="Linear Opmode")
@Config
public class TestMid extends LinearOpMode {


    SampleMecanumDrive drive;

    DcMotor brat,lift;
    Servo clawRotate,clawLeft,clawRight;
    int caz;

    DistanceSensor distanceSensor;
    Trajectory first;

    public static double clawLeftClose = 0.64;
    public static double clawLeftOpen = 0.3;
    public static double clawRightClose = 0.36;

    public static double preloadX = 53;
    public static double preloadY = 3.5;
    public static double preloadRot = 40;
    public static double preloadTg = 40;

    public static double midX = 49;
    public static double midY = 3.5;
    public static double midRot = 90+40;
    public static double midTg = 90+40;

    public static double precycleX = 51;
    public static double precycleY = -5;
    public static double precycleRot = 90;

    public static double cycle_x=52;
    public static double cycle_y = -21.2;
    public static double cycle_rot = 90;


    public static double  UNGHI_TG_1 = -90 - 130;
    public static double UNGHI_TG_2 = -90;
    Trajectory third;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //HardwareTesterInterpreter.initHWI(this,hardwareMap,telemetry,drive);

        clawRight = hardwareMap.get(Servo.class,"clawRight");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRotate = hardwareMap.get(Servo.class,"clawRotate");
        lift = hardwareMap.get(DcMotor.class,"lift");
        brat = hardwareMap.get(DcMotor.class,"brat");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        first = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                //.lineTo(new Vector2d(fX,fY))
                .lineTo(new Vector2d(31,1))
                .splineToSplineHeading(new Pose2d(preloadX, preloadY, Math.toRadians(preloadRot)),Math.toRadians(preloadTg))
                .build();



        waitForStart ();

        telemetry.addData("Facem cazul",caz);
        telemetry.update();

        while (opModeIsActive()) {
            main();
            break;
        }
    }



    void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (Exception ex){

        }
    }


    public void main(){
        drive.followTrajectory(first);
        drive.update();
        Trajectory second = drive.trajectoryBuilder(drive.getPoseEstimate(),Math.toRadians(-130))
                .splineToSplineHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)),Math.toRadians(-90))
                //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                .splineTo(new Vector2d(cycle_x,cycle_y),Math.toRadians(-90))
                //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                //.splineToLinearHeading()
                .build();
        drive.followTrajectory(second);
        drive.update();

        Wait(2000);

        third = drive.trajectoryBuilder(new Pose2d(cycle_x,cycle_y,Math.toRadians(cycle_rot)),false)
                .lineTo(new Vector2d(precycleX ,precycleY))
                .splineToSplineHeading(new Pose2d(midX,midY,Math.toRadians(midRot)),Math.toRadians(midTg))
                .build();
        drive.followTrajectory(third);

        Wait(2000);

        third = drive.trajectoryBuilder(drive.getPoseEstimate(),Math.toRadians(UNGHI_TG_1))
                .splineToSplineHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)),Math.toRadians(UNGHI_TG_2))
                //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                .splineTo(new Vector2d(cycle_x,cycle_y),Math.toRadians(-90))
                //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                //.splineToLinearHeading()
                .build();
        drive.followTrajectory(third);

    }
}
