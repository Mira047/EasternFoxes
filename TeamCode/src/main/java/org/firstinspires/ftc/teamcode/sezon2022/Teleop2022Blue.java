package org.firstinspires.ftc.teamcode.sezon2022;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Teleop2022Blue", group="Linear Opmode")
@Config
//s0Ft ArabEsc
public class Teleop2022Blue extends LinearOpMode {

    SampleMecanumDrive drive;
    Turret turret;
    DcMotor angle;
    DcMotor duck;
    DcMotor extender;
    Trajectory currentTrajectory;
    public static double speed = 0.66;
    public static double base_speed = 0.66;
    Servo intake1;
    Servo intake2;
    Servo claw;
    DistanceSensor distanceSensor;

    public static int angleUpPosition = -550;
    public static int angleMidPosition = -950;
    public static int antiRetardDown = -2800;
    public static int angle0 = 0;

    public static double clawClose = 1;
    public static double clawOpenIntake = 0.7;
    public static double clawOpenOut = 0.5;

    public static int extenderTresh = 20;

    public static int extenderUp = -1520;
    public static int extenderIntake = 0;

    public static double turretRot = 180-115;
    public static double turret0 = 180;

    int mm = 0;
    int u =0;

    int turretMode = 0;

    private void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (InterruptedException e){

        }
    }

    int x_use = 0;
    int up_use = 0;
    public static int antiRetardUp = 200;
    public static int fromAuto = 0;
    turret_rotation tr;
    GamepadUpdater gamepadUpdater;


    public static Pose2d startPose = new Pose2d();

    int freightInIntake = 0;

    public static double wX = -22.4;
    public static double wY = 40.1;

    Servo duckServo;

    @Override
    public void runOpMode(){
        if(fromAuto ==0)
            turret = new Turret(hardwareMap,1);
        else
            turret = new Turret(hardwareMap,0);
        turret.setPower(0.325);
        intake1 = hardwareMap.get(Servo.class,"intake1");
        intake2 = hardwareMap.get(Servo.class,"intake2");
        claw = hardwareMap.get(Servo.class,"claw");
        angle = hardwareMap.get(DcMotor.class,"angle");
        extender = hardwareMap.get(DcMotor.class,"extender");
        duck = hardwareMap.get(DcMotor.class,"duck");

        if(fromAuto == 0)
            angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setTargetPosition(angle.getCurrentPosition());
        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle.setPower(1);
        if(fromAuto == 0)
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(extender.getCurrentPosition());
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(1);
        mm =0;
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        startPose = new Pose2d();
        tr = new turret_rotation();
        gamepadUpdater = new GamepadUpdater();
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        claw.setPosition(clawOpenIntake);


        waitForStart();
        tr.start();
        gamepadUpdater.start();

        Thread t = new Thread(){
            @Override
            public void run(){
                while (opModeIsActive() && !isStopRequested()){
                    double d = distanceSensor.getDistance(DistanceUnit.MM);
                    if(d>=40){
                        freightInIntake = 0;
                    } else {
                        freightInIntake = 1;
                    }
                }
            }
        };
        t.start();


        while (opModeIsActive() && !isStopRequested()){
            if(gamepad1.left_bumper){

                intake1.setPosition(1);
                intake2.setPosition(0);


                if(gamepad2.b == false)
                    claw.setPosition(clawOpenIntake);
            } else if(gamepad1.left_trigger > 0){
                intake1.setPosition(0);
                intake2.setPosition(1);
            } else {
                intake1.setPosition(0.5);
                intake2.setPosition(0.5);
                if(gamepad2.b == false){
                    claw.setPosition(clawClose);
                }
            }
            speed = base_speed + base_speed * gamepad1.right_trigger;
            if(speed>1){
                speed = 1;
            }
            if(gamepad2.b){
                claw.setPosition(clawOpenOut);
                intake1.setPosition(1);
                intake2.setPosition(0);
            } else if(gamepad1.left_bumper == false && gamepad1.left_trigger == 0){
                intake1.setPosition(0.5);
                intake2.setPosition(0.5);
            }

            double a = 0;

            a =  a + 0.8 * (gamepad2.right_trigger - gamepad2.left_trigger);
            if(a!=0) {
                duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                duck.setPower(a);
            } else {
                duck.setPower(0);
                duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("angle",angle.getCurrentPosition());
            telemetry.addData("extender",extender.getCurrentPosition());
            telemetry.addData("turret",turret.getAngle() + " " +turret.getAngleHelper());
            telemetry.addData("collected",freightInIntake);
            telemetry.addData("turretMode",turretMode);
            telemetry.update();
            if(currentTrajectory == null){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speed,
                                -gamepad1.left_stick_x * speed,
                                -gamepad1.right_stick_x * speed
                        )
                );

                drive.update();
            }
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

    int freightMode = 0;

    void onGamepad1_A(){
        if(freightMode == 0){
            freightMode = 1;
        } else if(freightMode == 1){
            freightMode = 0;
        }
    }

    void onGamepad1_B(){

    }

    void onGamepad1_X(){
        if(turretMode == 0)
            turretMode = 1;
        else
            turretMode = 0;
    }

    void onGamepad1_Y(){

    }

    int extenderMode = 0;

    void onGamepad2_A(){
        if(extenderMode == 0){
            extenderMode = 1;
        } else {
            extenderMode = 0;
        }
    }

    void onGamepad2_B(){

    }

    void onGamepad2_X(){
        if(extender.getCurrentPosition() > - 300) {
            claw.setPosition(clawOpenIntake);
            turret.setPower(0.325);
            double offset = turret.getAngleHelper() - turret.getAngle();
            turret.setRotationAsync(turret0 - offset);
        }
    }

    void onGamepad2_Y(){
        /*
        if(u==0) {
            u = 1;
            Thread th = new Thread() {
                @Override
                public void run() {
                    float x = 0.41f;
                    int aa = 20;
                    for (int i = 0; i < 50; i++) {
                        Wait(10);
                        x += 0.01;
                        aa -= 1;
                        duck.setPower(x);
                    }
                    u = 0;
                }
            };
            th.start();
            mm -= 3000;
            duck.setPower(0.41f);
            duck.setTargetPosition(mm);
        }

         */
    }

    void onGamepad1_RB(){
        if(true || freightInIntake == 1) {
            turret.setPower(0.325);
            if (extenderMode == 0) {
                if(freightMode == 0) {
                    double offset = turret.getAngleHelper() - turret.getAngle();
                    angle.setTargetPosition(angleUpPosition);
                    turret.setRotationAsync(turretRot - offset);
                } else if(freightMode == 1) {
                    double offset = turret.getAngleHelper() - turret.getAngle();
                    angle.setTargetPosition(angleMidPosition);
                    turret.setRotationAsync(turretRot - offset);
                }
            } else {
                angle.setTargetPosition(angleMidPosition);
                double offset = turret.getAngleHelper() - turret.getAngle();
                turret.setRotationAsync(-turretRot);
            }
        }
    }

    void onGamepad1_LB(){

    }


    void onGamepad2_RB(){
        if(freightMode == 0) {
            extender.setTargetPosition(extenderUp);
        } else if(freightMode == 1) {
            extender.setTargetPosition(extenderUp - 300);
        }
    }

    void onGamepad2_LB(){
        extender.setTargetPosition(0);
    }

    private class GamepadUpdater implements Runnable {

        Thread localT;

        public void start(){
            if( localT == null ) {
                localT = new Thread(this);
                localT.start();
            }
        }

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

    private class turret_rotation implements Runnable{
        private Thread localT;

        int use_b=0;

        @Override
        public void run() {
            while(opModeIsActive() && !isStopRequested()){

                if(turretMode == 0) {
                    if (gamepad1.dpad_left) {
                        turret.setRotationAsync(turret.getTargetAngle() + 0.25);
                        Wait(25);
                    } else if (gamepad1.dpad_right) {
                        turret.setRotationAsync(turret.getTargetAngle() - 0.25);
                        Wait(25);
                    }
                    if (gamepad1.dpad_up) {
                        if (angle.getTargetPosition() + 5 <= antiRetardUp)
                            angle.setTargetPosition(angle.getTargetPosition() + 5);
                        Wait(10);
                    } else if (gamepad1.dpad_down) {
                        if (angle.getTargetPosition() - 5 >= antiRetardDown)
                            angle.setTargetPosition(angle.getTargetPosition() - 5);
                        Wait(10);
                    }
                } else {

                }
                if(gamepad2.dpad_left){
                    turret.setRotationAsync(turret.getTargetAngle()+1.5);
                    Wait(25);
                } if(gamepad2.dpad_right) {
                    turret.setRotationAsync(turret.getTargetAngle()-1.5);
                    Wait(25);
                }
                else if(gamepad2.dpad_up){
                    if(angle.getTargetPosition() + 15 <= antiRetardUp)
                        angle.setTargetPosition(angle.getTargetPosition()+15);
                    Wait(10);
                } else if(gamepad2.dpad_down){
                    if(angle.getTargetPosition() - 50 >= antiRetardDown)
                        angle.setTargetPosition(angle.getTargetPosition()-50);
                    Wait(10);
                }
                /*else if(gamepad2.right_bumper && use_b == 0){
                    use_b = 1;
                    extender.setTargetPosition(extenderUp);

                    //claw.setPosition(clawOpenOut);
                    use_b = 0;
                } else if(gamepad2.left_bumper && use_b == 0){
                    use_b = 1;
                    extender.setTargetPosition(0);

                    //claw.setPosition(clawOpenOut);
                    use_b = 0;
                }
                else if(gamepad1.right_bumper && use_b == 0){
                    use_b = 1;
                    angle.setTargetPosition(angleUpPosition);
                    turret.setRotation(turretRot);
                    //extender.setTargetPosition(extenderUp);
                    //while(extender.getCurrentPosition() != extender.getTargetPosition()){
                    //    Wait(1);
                    //}
                    //claw.setPosition(clawOpenOut);
                    use_b = 0;
                } else if(gamepad2.x && use_b ==0){
                    use_b = 1;
                    if(extender.getCurrentPosition() > -500) {
                        claw.setPosition(clawOpenIntake);
                        turret.setRotationAsync(turret0);
                    }
                    //angle.setTargetPosition(angle0);
                    //claw.setPosition(clawOpenIntake);
                    use_b = 0;
                }
                if(gamepad2.y && u == 0){
                    u = 1;
                    Thread th = new Thread() {
                        @Override
                        public void run() {
                            float x = 0.41f;
                            int aa=20;
                            for(int i=0;i<50;i++){
                                Wait(10);
                                x += 0.01;
                                aa -=1;
                                duck.setPower(x);
                            }
                            u=0;
                        }
                    };
                    th.start();
                    mm-=3000;
                    duck.setPower(0.41f);
                    duck.setTargetPosition(mm);
                    //duck.setPower(-0.9);
                }
                if(gamepad2.right_trigger >0 && u==0){
                    u=1;
                    Thread th = new Thread() {
                        @Override
                        public void run() {
                            Wait(300);
                            u=0;
                        }
                    };
                    th.start();
                    turretRot = turret.getAngle();
                }
                */
                /*\                else if(gamepad2.y){
                    extender.setTargetPosition(extender.getTargetPosition()-20);
                    Wait(50);
                } else if(gamepad2.a){
                    extender.setTargetPosition(extender.getTargetPosition()+20);
                    Wait(50);
                }
                 */
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
