package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sezon2022.CameraRecognition;

@Autonomous(name="TestGhearaParalela", group="Linear Opmode")
@Config
public class TestGhearaParalela extends LinearOpMode {

    SampleMecanumDrive drive;

    DcMotor brat,lift;
    Servo clawRotate,clawLeft,clawRight,ghidaj;
    int caz;

    DistanceSensor distanceSensor,distanceSensor2;
    Trajectory first;

    public static double fX = 15;
    public static double fY = 1.5;

    CameraRecognition cameraRecognition;

    public static double ghidajUp = 0.625 ;
    public static double ghidajMid = 0.4 + 0.1;
    public static double ghidajLow = 0.45 + 0.1;
    public static double ghidajIdle = 0.55 + 0.1 ;

    public static double clawRotateUp = 0.389; // 0.37;//0.2;
    public static double clawRotateUpFactor = 0.389;
    public static double clawRotateFactor = 0.001;

    public static double clawRotateMid = 0.372;
    public static double clawRotateLow = 0.325;
    public static double clawRotateLowFront = 0.5;
    public  static  double clawRotateInit = 0.506;//0.92;

    public static double clawLeftClose = 0.68;
    public static double clawLeftOpen = 0.37;

    public static double clawRightClose = 0.3;
    public static double clawRightOpen = 0.64;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //HardwareTesterInterpreter.initHWI(this,hardwareMap,telemetry,drive);

        clawRight = hardwareMap.get(Servo.class,"clawRight");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRotate = hardwareMap.get(Servo.class,"clawRotate");
        lift = hardwareMap.get(DcMotor.class,"lift");
        brat = hardwareMap.get(DcMotor.class,"brat");
        ghidaj = hardwareMap.get(Servo.class,"ghidaj");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        ghidaj.setPosition(ghidajIdle);
        ghidaj.setPosition(ghidajIdle);

        clawRotate.setPosition(0.55);

        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart ();

        brat.setTargetPosition(bratUp);
        while (opModeIsActive()) {
            int p = brat.getCurrentPosition();
            brat.setTargetPosition(bratUp);
            if(p >1900)
            {
                clawRotate.setPosition(clawRotateUp - (clawRotateFactor * (p - 1980)));
            }

        }
    }



    void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (Exception ex){

        }
    }

    public static double preloadX = 49; //47
    public static double preloadY = -2.9;
    public static double preloadRot = -36;
    public static double preloadTg = -36;

    public static double cycleX = 49.5;
    public static double cycleY = 0.5;
    public static double cycleRot = -35;
    public static double cycleTg = -35;

    public static double clawRotate_CYCLE =0.389;
    public static int bratUp = 1980;
    public static int liftUp = 1100;
    public static int liftUp_final = 0;

    private void setTargetPositionAfter(final String motor,final int target,final int timeout){
        Thread th = new Thread() {
            @Override
            public void run() {
                DcMotor dc = hardwareMap.get(DcMotor.class,motor);
                final int x = timeout;
                Wait(x);
                if(!opModeIsActive() || isStopRequested())
                    return;
                dc.setTargetPosition(target);
                dc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        };
        th.start();
    }

    private void setTargetPositionServoAfter(final String servo,final double target,final int timeout){
        Thread th = new Thread() {
            @Override
            public void run() {
                Servo s = hardwareMap.get(Servo.class,servo);
                final int x = timeout;
                Wait(x);
                if(!opModeIsActive() || isStopRequested())
                    return;
                s.setPosition(target);
            }
        };
        th.start();
    }


    double distance2D(Pose2d p1,Pose2d p2){
        double x = p1.getX()-p2.getX();
        double y= p1.getY() - p2.getY();
        return Math.sqrt(x*x+y*y) * 2.54;
    }


    private  void main(){


    }
}
