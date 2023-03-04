package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Auto2022Red", group="Linear Opmode")
@Config
public class Auto2022Red extends LinearOpMode {

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

    double clawIntake = 0.7;
    double clawClose = 1;
    double clawOpenOut = 0.5;

    CameraRecognition cameraRecognition;
    SampleMecanumDrive drive;
    Turret turret;
    DcMotor extender,angle;
    Servo claw;
    Servo release;
    DistanceSensor distanceSensor;
    Servo intake1,intake2;

    DcMotor slab;
    int freightInIntake=0;

    Trajectory trajEnd;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class,"claw");
        extender = hardwareMap.get(DcMotor.class,"extender");
        angle = hardwareMap.get(DcMotor.class,"angle");
        intake1 = hardwareMap.get(Servo.class,"intake1");
        slab = hardwareMap.get(DcMotor.class,"duck");
        intake2 = hardwareMap.get(Servo.class,"intake2");

        release = hardwareMap.get(Servo.class,"release");
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

        release.setPosition(0.4);

        trajEnd = drive.trajectoryBuilder(new Pose2d(0,-1,0))
                .lineToLinearHeading(new Pose2d(30,-2,Math.toRadians(0)))
                .build();


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

        while (!opModeIsActive()){
            telemetry.addData("ang",turret.getAngleHelper());
            telemetry.update();
        }


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
        waiting = 3;
        while (true && opModeIsActive() && !isStopRequested()){
            int p = angle.getCurrentPosition();
            int t = angle.getTargetPosition();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
        waiting = 0;
    }

    void waitForExtenderPosition(int tresh){
        waiting=2;
        int ms = 0;
        int lp=extender.getCurrentPosition();
        while (true && opModeIsActive() && !isStopRequested()){
            int p = extender.getCurrentPosition();
            int t = extender.getTargetPosition();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
            ms++;
            /*
            if(ms%40 == 0){
                int x = p-lp;
                if(x<5){
                    break;
                }
                lp = extender.getCurrentPosition();
            }

             */
        }
        waiting=0;
    }

    int waiting = 0;
    void waitForTurretAngle(double tresh){
        waiting = 1;
        while (true && opModeIsActive() && !isStopRequested()){
            double p = turret.getAngle();
            double t = turret.getTargetAngle();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
        waiting = 0;
    }

    public static int target_freight = 3;
    public static double[][] extraPosX = {{28},{34.5},{37.5},{42},{38},{-2},{-2}};
    public static double[][] extraPosY = {{-2},{-2},{-2},{-2},{-2},{-2},{-2}};
    public static double[][] extraPosRot = {{0},{0},{0},{0},{0},{0},{0}};
    public static double[] turretRotIntake = {0,0,0,0,0,0,0};
    public static double turretPower = 0.415;
    public static double turretPower2 = 0.415;
    Trajectory[] trajectories = new Trajectory[extraPosX.length];

    public static int freightCase = 3;

    void onFreightCollect(){

    }

    void onFreightRelease(){

    }

    /*
    void setExtenderTarget(int target){
        new Thread() {
            @Override
            public  void run() {
                extender.setTargetPosition(target);
                if (target > extender.getTargetPosition()) {
                    while (!(extender.getCurrentPosition() >= target - 50 && extender.getCurrentPosition() <= target + 50)) {
                        slab.setPower(-1);
                    }
                }
                slab.setPower(0);
            }
        }
    }

     */

    int currentFreight = 0;

    private void main(){


        currentFreight = 0;

        freightCase = cameraRecognition.getCase();

        Thread t = new Thread(){
            @Override
            public void run(){
                while (opModeIsActive() && !isStopRequested()){
                    double d = distanceSensor.getDistance(DistanceUnit.MM);
                    if(d>=50){
                        freightInIntake = 0;
                    } else {
                        freightInIntake = 1;
                    }
                    telemetry.addData("freight",freightInIntake);
                    telemetry.addData("waiting",waiting);
                    telemetry.update();
                }
            }
        };
        t.start();


        telemetry.addLine("Case: " + Integer.toString(freightCase));
        telemetry.update();

        Teleop2022_v2.fromAuto = 1;

        double d=0;

        int ext = -1300;
        int ang = -750;

        double int1 = 0.45;
        double int2= 0.55;

        double turr = 117;

        if(freightCase == 1){
            ang = -800;
            ext = -1350;
            int1=0.25;
            int2=0.75;
            turr = 115.25;
        } else if(freightCase == 2){
            ang = -1120;
            ext = -1100;
            turr = 114.5;
            int1=0.3;
            int2=0.7;
        } else if(freightCase == 3) {
            ext = -1350;
            ang = -800;
            int1=0.45;
            int2=0.55;
            turr = 114.35;
        }

        claw.setPosition(clawClose);
        Wait(100);
        angle.setTargetPosition(ang);
        turret.setRotationAsync(turr);
        waitForTurretAngle(1.5);
        waitForAnglePosition(100);
        extender.setTargetPosition(ext);

        waitForExtenderPosition(75);
        claw.setPosition(clawOpenOut);
        intake1.setPosition(0.6);
        intake2.setPosition(0.4);
        while (freightInIntake != 0){
            Wait(0);
        }
        turret.setPower(turretPower);
        Wait(20);
        extender.setTargetPosition(0);
        waitForExtenderPosition(150);
        if(freightCase != 3) {
            angle.setTargetPosition(-800);
            waitForAnglePosition(50);
        }
        //cap_y.setPosition(0.8);
        turret.setPower(turretPower);
        turret.setRotationAsync(turretRotIntake[0]);

        waitForTurretAngle(20 - Math.abs(turretRotIntake[0]));
        turret.setPower(turretPower);
        angle.setTargetPosition(-2400);

        claw.setPosition(clawIntake);
        intake1.setPosition(1);
        intake2.setPosition(0);


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
            intake1.setPosition(1);
            intake2.setPosition(0);
            int temp = 0;
            while(freightInIntake != 1 && !isStopRequested() && opModeIsActive()) {
                if (drive.isBusy() == false) {

                    if (temp == 0) {
                        temp = 1;
                        turret.setRotationAsync(-10);
                    }
                    double a = turret.getAngle();
                    double d1 = Math.abs(10 - a);
                    if (d1 < 1) {
                        turret.setRotationAsync(-10);
                    }
                    double d2 = Math.abs(-10 - a);
                    if (d2 < 1) {
                        turret.setRotationAsync(10);
                    }
                }

                drive.update();
            }
            Wait(150);
            if(isStopRequested() || opModeIsActive() == false){
                break;
            }
            drive.breakFollowing();
            drive.setDrivePower(new Pose2d());
            drive.update();
            intake1.setPosition(0.5);
            intake2.setPosition(0.5);
            Wait(200);
            //intake.setPower(0);
            telemetry.addLine("Done collecting");
            telemetry.update();
            angle.setTargetPosition(-800);
            claw.setPosition(clawClose);
            //intake.setPower(-0.1);
            waitForAnglePosition(70);
            turret.setPower(turretPower2);
            turret.setRotationAsync(114+d );
            waitForTurretAngle(2);
            d+=1;

            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(0,-2,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(traj);

            double r = Math.toDegrees(drive.getPoseEstimate().getHeading());
            if(r > 180){
                r = r - 360;
            }
            double offset = turret.getAngleHelper() - turret.getAngle();
            turret.setRotationAsync(114 + d - r );

            telemetry.addData("offset",offset);
            telemetry.update();

            waitForTurretAngle(1);
            extender.setTargetPosition(ext - (int)(d * 30));
            waitForAnglePosition(70);
            waitForExtenderPosition(75);
            claw.setPosition(clawOpenOut);
            while (opModeIsActive() && !isStopRequested() && freightInIntake == 1){
                intake1.setPosition(0.7);
                intake2.setPosition(0.3);
            }
            if(isStopRequested() || opModeIsActive() == false){
                break;
            }
            Wait(250);
            extender.setTargetPosition(0);
            waitForExtenderPosition(200);
            turret.setPower(turretPower);
            turret.setRotationAsync(turretRotIntake[currentFreight]);
            if(o < target_freight) {
                waitForTurretAngle(20 - Math.abs(turretRotIntake[currentFreight]));
                angle.setTargetPosition(-2400);
            }
            claw.setPosition(clawIntake);
            currentFreight += 1;
            //HardwareTesterInterpreter.interpretScript(scriptHighFreight,"base64");
        }

        if(isStopRequested() || opModeIsActive() == false){
            return;
        }

        intake1.setPosition(0.5);
        intake2.setPosition(0.5);
        turret.setRotationAsync(0);

        drive.followTrajectoryAsync(trajEnd);

        int tt = 0;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            if(tt == 0) {
                double a = turret.getAngle();
                if (a <= 20) {
                    tt = 1;
                    angle.setTargetPosition(-1700);
                }
            }
        }

        Teleop2022_v2.startPose = drive.getPoseEstimate();
    }
}
