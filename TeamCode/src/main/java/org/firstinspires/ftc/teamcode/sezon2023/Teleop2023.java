package org.firstinspires.ftc.teamcode.sezon2023;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Teleop2023", group="Linear Opmode")
@Config
//s0Ft ArabEsc
public class Teleop2023 extends LinearOpMode {

    SampleMecanumDrive drive;
    DistanceSensor distanceSensor;

    public static int corectieLimita = 350;
    public static double speed = 1;
    public static double base_speed = 1;

    public static boolean driveIsBusy=false;

    public static int liftUp = 1280;
    public static int liftMid = 260; //1050;
    public static int liftLow = 850;
    public static int liftLowFront = 1150;

    public static double oldBratRPM = 43;
    public static double BratRPM = 43;
//forta
    public static double speedRotateFactor = 0.7;

    public static int liftRetard = 1400;
    public static int bratPickRetardCone = 250;

    public static int liftDown = 0;
    public static int liftUpStep = 40;
    public static double clawRotateStep = 0.002;

    public static double clawLeftClose = 0.64;
    public static double clawRightClose = 0.36;
    public static double clawLeftOpen = 0.43;
    public static double clawRightOpen = 0.6;

    public static double bratPowerUp = 1;
    public static double bratPowerDown= 0.7;

    public static double corectieVitezaTeleop=0.4;

    public static double clawRotateUp = 0.38; // 0.37;//0.2;
    public static double clawRotateMid = 0.377;
    public static double clawRotateLow = 0.335;
    public static double clawRotateLowFront = 0.514;
    public  static  double clawRotateInit = 0.517;//0.92;

    public static int autoPutDelay = 300;

    public  static  int bratUpPosition = 2050;// 2050;
    public  static  int bratMidPosition = 2200; //2345;
    public  static  int bratLowPosition = 2650;

    public static double ghidajUp = 0.57;
    public static double ghidajMid = 0.55;
    public static double ghidajLow = 0.5;
    public static double ghidajIdle = 0.25; // 0.45

    public static int bratLowPositionFront = 300;

    public static int liftPickRetardCone = 100;
    public static double clawRotatePickRetardCone = 0.46;

    public  static int bratDownPosition = 0;

    public static int stateMechanism = 0;

    public static int MECHANISM_IDLE =0;
    public static int MECHANISM_INUSE = 1;
    public static int MOD_FIXARE = 0;

    boolean liftState = false;

    public static double ghidajLift=0.8;

    Servo clawRight,clawLeft,clawRotate,servoEncoderFront,servoEncoderRight,servoEncoderLeft,ghidaj;
    DcMotor lift,brat;
    boolean clawState = false;


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

        stateMechanism = MECHANISM_IDLE;
        driveIsBusy = false;
        gamepadUpdater = new GamepadUpdater();
        liftUpdater = new LiftUpdater();

        clawRight = hardwareMap.get(Servo.class,"clawRight");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRotate = hardwareMap.get(Servo.class,"clawRotate");
        servoEncoderFront = hardwareMap.get(Servo.class,"encoderServo3");
        servoEncoderRight = hardwareMap.get(Servo.class,"encoderServo1");
        servoEncoderLeft = hardwareMap.get(Servo.class,"encoderServo2");
        lift = hardwareMap.get(DcMotor.class,"lift");
        brat = hardwareMap.get(DcMotor.class,"brat");
        ghidaj = hardwareMap.get(Servo.class,"ghidaj");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        servoEncoderFront.setPosition(0.6);
        servoEncoderRight.setPosition(0.6);
        servoEncoderLeft.setPosition(0.7);

        ghidaj.setPosition(ghidajIdle);

        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(bratPowerUp);
        brat.setDirection(DcMotorSimple.Direction.FORWARD);

        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        clawRotate.setPosition(clawRotateInit);

        clawState = false;

        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        gamepadUpdater.start();
        liftUpdater.start();
        lift.setTargetPosition(liftDown);


        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("brat position",brat.getCurrentPosition());
            telemetry.addData("brat target",brat.getTargetPosition());
            telemetry.update();
            speed = base_speed + base_speed * gamepad2.right_trigger - (base_speed/2) * gamepad2.left_trigger;
            if(speed>1){
                speed = 1;
            }
            if(!drive.isBusy() && driveIsBusy == false && MOD_FIXARE == 0) {
                if (gamepad2.dpad_left){
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                     speed,
                                    -gamepad2.right_stick_x * speed * speedRotateFactor
                            )
                    );
                } else if  (gamepad2.dpad_right){
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    -speed,
                                    -gamepad2.right_stick_x * speed * speedRotateFactor
                            )
                    );
                }
                else if (gamepad2.dpad_up){
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    speed,
                                    0,
                                    -gamepad2.right_stick_x * speed * speedRotateFactor
                            )
                    );
                } else if (gamepad2.dpad_down){
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -speed,
                                    0,
                                    -gamepad2.right_stick_x * speed * speedRotateFactor
                            )
                    );
                }
                else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad2.left_stick_y * speed,
                                    -gamepad2.left_stick_x * speed,
                                    -gamepad2.right_stick_x * speed * speedRotateFactor
                            )
                    );
                }
            }
                drive.update();


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
    boolean gm2_l3 = false;
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
        if(brat.getTargetPosition() == 0){

        } else {

        }
        double d = clawRotate.getPosition();
        if(stateMechanism == MECHANISM_IDLE){
            stateMechanism = MECHANISM_INUSE;
            brat.setPower(bratPowerUp);
            brat.setTargetPosition((int)Math.floor(bratLowPosition * oldBratRPM/BratRPM));
        //if(d >= clawRotateInit-0.02 ){
            clawRotate.setPosition(clawRotateLow);
        } else {
            stateMechanism = MECHANISM_IDLE;
            clawRotate.setPosition(clawRotateInit);
            brat.setPower(bratPowerDown);
            brat.setTargetPosition(bratDownPosition);
        }
        if(liftState == false) {
            liftState=true;
            lift.setTargetPosition(liftLow);
        } else {
            liftState=false;
            lift.setTargetPosition(liftDown);
        }
    }

    void onGamepad1_A_(){

    }


    void onGamepad1_B(){
        /*
        if(clawState) {
            clawLeft.setPosition(clawLeftClose);
            clawRight.setPosition(clawRightClose);
            clawState = false;
        }
        else {
            clawLeft.setPosition(clawLeftOpen);
            clawRight.setPosition(clawRightOpen);
            clawState = true;
        }

         */
    }

    void onGamepad1_X_(){
        if(liftState == false) {
            liftState=true;
            lift.setTargetPosition(liftUp);
        } else {
            liftState=false;
            lift.setTargetPosition(liftDown);
        }
    }

    void onGamepad1_X(){

        /*
        if(brat.getTargetPosition() == 0){
            brat.setPower(bratPowerUp);
            brat.setTargetPosition(bratMidPosition);
        } else {
            brat.setPower(bratPowerDown);
            brat.setTargetPosition(bratDownPosition);
        }

         */
        double d = clawRotate.getPosition();
        if(stateMechanism == MECHANISM_IDLE){
            stateMechanism = MECHANISM_INUSE;
            brat.setPower(bratPowerUp);
            brat.setTargetPosition((int)Math.floor(bratMidPosition * oldBratRPM/BratRPM));
            ghidaj.setPosition(ghidajMid);
        //if(d >=
            // Init-0.02 ){
            clawRotate.setPosition(clawRotateMid);
        } else {
            stateMechanism = MECHANISM_IDLE;
            clawRotate.setPosition(clawRotateInit);
            brat.setPower(bratPowerDown);
            brat.setTargetPosition(bratDownPosition);
            ghidaj.setPosition(ghidajIdle);
        }
        if(liftState == false) {
            liftState=true;
            lift.setTargetPosition(liftMid);
        } else {
            liftState=false;
            lift.setTargetPosition(liftDown);
        }


    }

    void onGamepad1_Y_(){
        double d = clawRotate.getPosition();
        if(stateMechanism == MECHANISM_IDLE){
            stateMechanism = MECHANISM_INUSE;

                brat.setPower(bratPowerUp);
                brat.setTargetPosition((int)Math.floor(bratUpPosition * oldBratRPM/BratRPM));
                ghidaj.setPosition(ghidajUp);

        //if(d >= clawRotateInit-0.02 ){
            clawRotate.setPosition(clawRotateUp);
        } else {
            stateMechanism = MECHANISM_IDLE;
            brat.setPower(bratPowerDown);
            brat.setTargetPosition(bratDownPosition);
            clawRotate.setPosition(clawRotateInit);
            ghidaj.setPosition(ghidajIdle);
        }
    }

    void onGamepad2_A(){
        lift.setTargetPosition(liftPickRetardCone);
        clawRotate.setPosition(clawRotatePickRetardCone);
        brat.setTargetPosition(bratPickRetardCone);
    }



    void onGamepad2_LB(){

    }

    void onGamepad2_RB(){
        if(stateMechanism == MECHANISM_INUSE) {
            brat.setTargetPosition(brat.getTargetPosition() + 425);

            int tresh = 20;
            int p = brat.getCurrentPosition();
            int t = brat.getTargetPosition();
            while (!(p >= t - tresh && p <= t + tresh)) {
                p = brat.getCurrentPosition();
            }

            clawLeft.setPosition(clawLeftOpen);
            clawRight.setPosition(clawRightOpen);

            Wait(30);
            clawRotate.setPosition(clawRotateInit);

            driveIsBusy = true;
            drive.setWeightedDrivePower(new Pose2d(-1, 0, 0));
            drive.update();
            Wait(autoPutDelay);
            driveIsBusy=false;
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            drive.update();

            stateMechanism = MECHANISM_IDLE;
            brat.setPower(bratPowerDown);
            brat.setTargetPosition(bratDownPosition);
            clawRotate.setPosition(clawRotateInit);
            ghidaj.setPosition(ghidajIdle);
                liftState=false;
                lift.setTargetPosition(liftDown);

        }
    }

    void onGamepad2_Y(){

    }

    void onGamepad1_RB(){
        if(clawRotate.getPosition() >=0.5 && clawRotate.getPosition() <= 0.52){
            clawRotate.setPosition(0.565);
        } else {
            clawRotate.setPosition(clawRotateInit);
        }
    }


    void onGamepad1_LB(){
        if(brat.getTargetPosition() == 0){

        } else {

        }
        double d = clawRotate.getPosition();
        if(stateMechanism == MECHANISM_IDLE){
            stateMechanism = MECHANISM_INUSE;
            brat.setPower(bratPowerUp);
            brat.setTargetPosition((int)Math.floor(bratLowPositionFront * oldBratRPM/BratRPM));
            ghidaj.setPosition(ghidajLift);
        //if(d >= clawRotateInit-0.02 ){
            clawRotate.setPosition(clawRotateLowFront);
        } else {
            stateMechanism = MECHANISM_IDLE;
            clawRotate.setPosition(clawRotateInit);
            brat.setPower(bratPowerDown);
            brat.setTargetPosition(bratDownPosition);
            ghidaj.setPosition(ghidajIdle);
        }
        if(liftState == false) {
            liftState=true;
            lift.setTargetPosition(liftLowFront);
        } else {
            liftState=false;
            lift.setTargetPosition(liftDown);
        }
    }

    void corectie(double directie){
        double dist = 1000;
        long startTime = System.currentTimeMillis();
        long startTimeReal = startTime;
        driveIsBusy=true;
        while (dist > corectieLimita && opModeIsActive() && !isStopRequested()){
            long t2 = System.currentTimeMillis();
            if(t2 - startTimeReal >=400){

                telemetry.addData("Distanta de fail",dist);
                telemetry.update();
                break;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            corectieVitezaTeleop  * directie
                    )
            );
            drive.update();
            dist = distanceSensor.getDistance(DistanceUnit.MM);
        }
        driveIsBusy=false;
        drive.setWeightedDrivePower(new Pose2d(0,0,0));


        telemetry.addData("Corectat cu",dist);
        telemetry.update();
    }

    void onGamepad2_B(){
        //drive.turn(Math.toRadians(-90));
        //drive.update();
        //brat.setTargetPosition(brat.getTargetPosition() + 250);
        corectie(-1.2);
    }

    void onGamepad2_X(){
        //brat.setTargetPosition(brat.getTargetPosition() - 250);
        corectie(1.2);
        //drive.turn(Math.toRadians(90));
        //drive.update();
    }

    void onGamepad2_R3(){
        if(MOD_FIXARE == 0) {
            MOD_FIXARE = 1;
        } else {
            MOD_FIXARE = 0;
            brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brat.setTargetPosition(0);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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
                if(gamepad1.b){
                    clawLeft.setPosition(clawLeftOpen);
                    clawRight.setPosition(clawRightOpen);

                }
                else {
                    clawLeft.setPosition(clawLeftClose);
                    clawRight.setPosition(clawRightClose);
                }



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
                if(gamepad2.right_stick_button && !gm2_l3){
                    onGamepad2_R3();
                }

                gm2_l3 = gamepad2.right_stick_button;
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

                if(MOD_FIXARE == 0) {
                    if (gamepad1.dpad_up) {
                        if (lift.getTargetPosition() + 5 <= liftRetard)
                            lift.setTargetPosition(lift.getTargetPosition() + liftUpStep);
                        Wait(10);
                    } else if (gamepad1.dpad_down) {
                        if (lift.getTargetPosition() - liftUpStep >= 0)
                            lift.setTargetPosition(lift.getTargetPosition() - liftUpStep);
                        else
                            lift.setTargetPosition(0);
                        Wait(10);
                    }

                    if (gamepad1.dpad_left) {
                        if (clawRotate.getPosition() + clawRotateStep <= clawRotateInit)
                            clawRotate.setPosition(clawRotate.getPosition() + clawRotateStep);
                        Wait(10);
                    } else if (gamepad1.dpad_right) {
                        if (clawRotate.getPosition() - clawRotateStep >= clawRotateUp - 0.2)
                            clawRotate.setPosition(clawRotate.getPosition() - clawRotateStep);
                        Wait(10);
                    }
                }
                 else {
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

//ionut e tataie fortza si fute de rupe
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

