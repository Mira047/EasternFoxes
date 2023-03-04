package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Fixare teleop belita", group="Linear Opmode")
@Config
//s0Ft ArabEsc
public class Teleop2023_retard extends LinearOpMode {

    public static double speed = 0.75;
    public static double base_speed = 0.75;

    public static int liftUp = 1090;
    public static int liftMid = 1050;
    public static int liftLow = 1000;
    public static int liftLowFront = 1300;
    //forta
    public static double speedRotateFactor = 1;

    public static int liftRetard = 1400;

    public static int liftDown = 0;
    public static int liftUpStep = 25;
    public static double clawRotateStep = 0.002;

    public static double clawLeftClose = 0.585;
    public static double clawRightClose = 0.44;
    public static double clawLeftOpen = 0.3;
    public static double clawRightOpen = 0.7;

    public static double bratPowerUp = 0.6;
    public static double bratPowerDown= 0.7;

    public static double clawRotateUp = 0.37;//0.2;
    public static double clawRotateMid = 0.345;
    public static double clawRotateLow = 0.325;
    public static double clawRotateLowFront = 0.5;
    public  static  double clawRotateInit = 0.508;//0.92;

    public  static  int bratUpPosition = 2050;
    public  static  int bratMidPosition = 2345;
    public  static  int bratLowPosition = 2650;
    public static int bratLowPositionFront = 300;

    public  static int bratDownPosition = 0;

    public static int stateMechanism = 0;

    public static int MECHANISM_IDLE =0;
    public static int MECHANISM_INUSE = 1;

    Servo clawRight,clawLeft,clawRotate;
    DcMotor lift,brat;

    private void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (InterruptedException e){

        }
    }

    GamepadUpdater gamepadUpdater;
    LiftUpdater liftUpdater;


    @Override
    public void runOpMode(){

        gamepadUpdater = new GamepadUpdater();
        liftUpdater = new LiftUpdater();

        clawRight = hardwareMap.get(Servo.class,"clawRight");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRotate = hardwareMap.get(Servo.class,"clawRotate");
        lift = hardwareMap.get(DcMotor.class,"lift");
        brat = hardwareMap.get(DcMotor.class,"brat");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);


        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(bratPowerUp);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        clawRotate.setPosition(clawRotateInit);


        waitForStart();

        gamepadUpdater.start();
        liftUpdater.start();
        lift.setTargetPosition(liftDown);


        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("brat position",brat.getCurrentPosition());
            telemetry.addData("brat target",brat.getTargetPosition());
            telemetry.update();

        }
    }

    //States
    boolean gm1_a=false;
    boolean gm1_b=false;
    boolean gm1_x=false;
    boolean gm1_y=false;
    boolean gm2_a=false;
    boolean gm2_b=false;
    boolean gm2_x=false;
    boolean gm2_y=false;
    boolean gm1_rightbumper=false;
    boolean gm1_leftbumper=false;
    boolean gm2_rightbumper=false;
    boolean gm2_leftbumper=false;

    void onGamepad1_Y(){
        onGamepad1_A_();
        onGamepad1_Y_();
        onGamepad1_X_();
    }

    void onGamepad1_A(){

    }

    void onGamepad1_A_(){

    }

    void onGamepad1_B(){

    }

    void onGamepad1_X_(){

    }

    void onGamepad1_X(){

    }

    void onGamepad1_Y_(){

    }

    void onGamepad2_A(){

    }

    void onGamepad2_B(){

    }

    void onGamepad2_X(){

    }

    void onGamepad2_Y(){

    }

    void onGamepad1_RB(){

    }

    void onGamepad1_LB(){

    }


    void onGamepad2_RB(){

    }

    void onGamepad2_LB(){

    }

    private class GamepadUpdater implements Runnable {

        Thread localT;

        public void start(){
            if( localT == null ) {
                localT = new Thread(this);
                localT.start();
            }
        }

        int gtoggle=0;
        @Override
        public void run(){
            while (opModeIsActive() && !isStopRequested()){


                if(gamepad1.a && !gm1_a){
                    //Press
                    onGamepad1_A();
                }
                if(gamepad1.x && !gm1_x){
                    //Press
                    onGamepad1_X();
                }
                if(gamepad1.y && !gm1_y){
                    //Press
                    onGamepad1_Y();
                }
                if(gamepad1.b && !gm1_b){
                    //Press
                    onGamepad1_B();
                }
                if(gamepad2.a && !gm2_a){
                    //Press
                    onGamepad2_A();
                }
                if(gamepad2.x && !gm2_x){
                    //Press
                    onGamepad2_X();
                }
                if(gamepad2.y && !gm2_y){
                    //Press
                    onGamepad2_Y();
                }
                if(gamepad2.b && !gm2_b){
                    //Press
                    onGamepad2_B();
                }
                if(gamepad1.right_bumper && !gm1_rightbumper){
                    //Press
                    onGamepad1_RB();
                }
                if(gamepad1.left_bumper && !gm1_leftbumper){
                    //Press
                    onGamepad1_LB();
                }
                if(gamepad2.right_bumper && !gm2_rightbumper){
                    //Press
                    onGamepad2_RB();
                }
                if(gamepad2.left_bumper && !gm2_leftbumper){
                    //Press
                    onGamepad2_LB();
                }
                gm1_a = gamepad1.a;
                gm1_b = gamepad1.b;
                gm1_x = gamepad1.x;
                gm1_y = gamepad1.y;
                gm2_a = gamepad2.a;
                gm2_b = gamepad2.b;
                gm2_x = gamepad2.x;
                gm2_y = gamepad2.y;
                gm1_rightbumper = gamepad1.right_bumper;
                gm1_leftbumper = gamepad1.left_bumper;
                gm2_leftbumper = gamepad2.left_bumper;
                gm2_rightbumper = gamepad2.right_bumper;
            }
        }
    }

    private class LiftUpdater implements Runnable{
        private Thread localT;



        @Override
        public void run() {
            while(opModeIsActive() && !isStopRequested()){

                if (gamepad2.dpad_up) {
                    //if (lift.getTargetPosition() + 5 <= liftRetard)
                        lift.setTargetPosition(lift.getTargetPosition() + liftUpStep);
                    Wait(10);
                } else if (gamepad2.dpad_down) {
                    //if (lift.getTargetPosition() - liftUpStep >= 0)
                        lift.setTargetPosition(lift.getTargetPosition() - liftUpStep);
                    //else
                    //    lift.setTargetPosition(0);
                    Wait(10);
                }

                if (gamepad2.dpad_left) {
                    brat.setTargetPosition(brat.getTargetPosition() + liftUpStep);
                    Wait(10);
                } else if (gamepad2.dpad_right) {
                    brat.setTargetPosition(brat.getTargetPosition() - liftUpStep);
                    Wait(10);
                }



            }
        }
        public void start(){
            if( localT == null ) {
                localT = new Thread(this);
                localT.start();
            }
        }
    }
}

