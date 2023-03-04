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
import org.firstinspires.ftc.teamcode.sezon2022.CameraRecognition;
import org.firstinspires.ftc.teamcode.sezon2022.PipeLineDetector;

@Autonomous(name="NationalaDreaptaHigh it39", group="Linear Opmode")
@Config
public class NationalaDreaptaHigh extends LinearOpMode {

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
    public static double clawRotateMid = 0.372;
    public static double clawRotateLow = 0.325;
    public static double clawRotateLowFront = 0.5;
    public  static  double clawRotateInit = 0.506;//0.92;

    void parcare(int cycle){
        brat.setTargetPosition(0);
        lift.setTargetPosition(0);
        clawRotate.setPosition(0.56);
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);

        if(!opModeIsActive() || isStopRequested())
            return;

        if(caz == 1){
            Trajectory parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(49.5 + cycle * 0.1,25,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(parcare);
            drive.update();
            Pose2d cPos = drive.getPoseEstimate();
            if(distance2D(cPos,new Pose2d(49.5 + cycle * 0.1,21)) > 30){
                parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(51 + cycle * 0.4,25,Math.toRadians(0)))
                        .build();
                drive.followTrajectory(parcare);
                drive.update();
            }
            //drive.turn(Math.toRadians(-90));
        } else if(caz == 2){
            Trajectory parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(46,0,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(parcare);
        } if(caz == 3){

            Trajectory parcare = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                    .splineToSplineHeading(new Pose2d(precycleX + Vericu[cycle],precycleY,Math.toRadians(precycleRot)),Math.toRadians(-90))
                    //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                    .splineTo(new Vector2d(cycle_x + Vericu[cycle],cycle_y+1.2),Math.toRadians(-90))
                    //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                    //.splineToLinearHeading()
                    .build();
                    /*drive.trajectoryBuilder(drive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(cycle_x,cycle_y,Math.toRadians(cycle_rot)))
                    .build();

                     */
            drive.followTrajectory(parcare);
        }


    }

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

        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class,"distanceSensor2");

        first = //drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                //.lineTo(new Vector2d(fX,fY))
                //.splineToSplineHeading(new Pose2d(preloadX, preloadY, Math.toRadians(preloadRot)),Math.toRadians(preloadTg)).build();

                drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)),false)
                        //.lineTo(new Vector2d(fX,fY))
                        .lineTo(new Vector2d(25,1))
                        .splineToSplineHeading(new Pose2d(preloadX-1.1, preloadY, Math.toRadians(preloadRot)),Math.toRadians(preloadTg))
                        .build();

        cameraRecognition = new CameraRecognition(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);

        while(!opModeIsActive() && !isStopRequested()){
            if(cameraRecognition.detector.caz == PipeLineDetector.Status.VERDE1){
                caz = 1;
            }
            else if(cameraRecognition.detector.caz == PipeLineDetector.Status.ROSU2){
                caz = 2;
            } else if(cameraRecognition.detector.caz == PipeLineDetector.Status.ALBASTRU3){
                caz = 3;
            }
            telemetry.addData("detected", caz);
            telemetry.update();

        }

        waitForStart ();

        cameraRecognition.stop();
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

    public static double preloadX = 49; //47
    public static double preloadY = 2.9;
    public static double preloadRot = 35;
    public static double preloadTg = 35;

    public static double cycleX = 52;
    public static double cycleY = 2.9;
    public static double cycleRot = 40;
    public static double cycleTg = 40;


    public static double sensorretard = 167;
    public static double corectieLimita = 245;
    public static double corectieDirectie = -1;
    public static int corectieTimeoutChange = 150;


    public static int CYCLES = 5;

    /*
    1 pentru stanga
    -1 pentru dreapta
     */

    public static double corectieViteza = 0.32;
    public static double corectieVitezaRot = 0.28;

    public static double clawRotate_CYCLE =0.389;
    public static int bratUp = 1965;
    public static int liftUp = 1100;
    public static int liftUp_final = 0;

    public static int[] LIFT_CYCLES = {
            850,
            694,
            445,
            310,
            0
    };

    public static double[] Vericu = {
            0.5,
            1.3,
            2,
            2.5,
            3.5
    };
    public static double[] VericuY = {
        0,
        0,
        0,
        0.2,
        1.0
    };

    public static double[] VericuRotire = {
            -1,
            -1.5,
            -1.5,
            -1.3,
            -0.5
    };

    public static double cycle_final_x = 54;
    public static double cycle_final_y = 3.2;
    public static double cycle_final_rot =50;

    Pose2d cycle_final;

    public static double cycle_x=49;
    public static double cycle_y = -24.5;
    public static double cycle_rot = 90;

    public static double precycleX = 50;
    public static double precycleY = -7;
    public static double precycleRot = 90;

    public static double cycle_x_init=52;
    public static double cycle_y_init = -23.2;
    public static double cycle_rot_init = 90;


    public static double clawLeftClose = 0.68;
    public static double clawLeftOpen = 0.37;

    public static double clawRightClose = 0.3;
    public static double clawRightOpen = 0.64;

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
    public static double clawRotateFactor =0.00006;

    double distance2D(Pose2d p1,Pose2d p2){
        double x = p1.getX()-p2.getX();
        double y= p1.getY() - p2.getY();
        return Math.sqrt(x*x+y*y) * 2.54;
    }


    public static int autoPutDelay = 250;//280;
    void puneCon(int cycle){
        int targ = brat.getTargetPosition() + 550;
        brat.setTargetPosition(brat.getTargetPosition() + 550);

        int tresh = 20;
        int p = brat.getCurrentPosition();
        int initPos = p;
        int t = brat.getTargetPosition();
        long sTime = System.currentTimeMillis();
        int plast = brat.getCurrentPosition();
        while (!(p >= t - tresh && p <= t + tresh)) {
            p = brat.getCurrentPosition();
            long cTime = System.currentTimeMillis();
            if(cTime-sTime > 100){
                sTime = cTime;
                if(Math.abs(p-plast) < 20){
                    // Brat blocat
                    int cPos = brat.getCurrentPosition();
                    brat.setTargetPosition(bratUp-30);
                    drive.setWeightedDrivePower(new Pose2d(0,1,0));
                    Wait(215);
                    drive.setWeightedDrivePower(new Pose2d(0,0,0));
                    brat.setTargetPosition(targ);
                }
                plast = p;
            }
            clawRotate.setPosition(clawRotateUp - (clawRotateFactor * (p - 1980)));
            if(Math.abs(p-initPos) > 100){
                double pos = Math.max(clawRotateUp - 0.001 * ((Math.abs(p-initPos)-100)/27),clawRotateUp-0.013);
                // clawRotate.setPosition(pos);
            }
            /*
            if(Math.abs(p-initPos) > 100){
                double pos = Math.max(clawRotateUp - 0.001 * ((Math.abs(p-initPos)-100)/27),clawRotateUp-0.013);
                clawRotate.setPosition(pos);
            }*/
            Wait(3);
        }
        clawLeft.setPosition(clawLeftOpen);
        clawRight.setPosition(clawRightOpen);
        Wait(80);
        clawRotate.setPosition(0.512);

        if(cycle < 5){
            /*
            if(LIFT_CYCLES[cycle] <600)
            lift.setTargetPosition(LIFT_CYCLES[cycle]);
            else{

             */
                lift.setTargetPosition(600);
            //}
        }


        drive.setWeightedDrivePower(new Pose2d(-1, 0, 0));
        drive.update();
        Wait(autoPutDelay);
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.update();


    }

    void setMecanismeDupaCon(int cycle){
        brat.setTargetPosition(0);
        lift.setTargetPosition(LIFT_CYCLES[cycle]);
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        if(cycle==4){
            clawRotate.setPosition(0.518);
        }
        else {
            clawRotate.setPosition(0.522);
        }
        ghidaj.setPosition(ghidajIdle);
    }

    public static double ghidajLift=0.8;

    public static double distanceReset = 8;

    public long ProgramStartTime;
    private  void main(){
        ProgramStartTime = System.currentTimeMillis();
        int cycle = 0;
        cycle_x = cycle_x_init;
        cycle_y = cycle_y_init;
        cycle_rot = cycle_rot_init;
        cycle_final = new Pose2d(cycle_final_x,cycle_final_y,Math.toRadians(cycle_final_rot));
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        lift.setTargetPosition(liftUp);

        setTargetPositionAfter("brat",bratUp,200);
        setTargetPositionServoAfter("clawRotate",clawRotateUp,600); //0.362
        ghidaj.setPosition(ghidajUp);
        drive.followTrajectory(first);


        brat.setTargetPosition(bratUp);

        lift.setTargetPosition(liftUp);

        drive.setWeightedDrivePower(new Pose2d(1,0,0));
        drive.update();
        Wait(200);
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        drive.update();

        puneCon(0);

        int lastCycle = 0;
        for(int currentCycle = 0;currentCycle<5;currentCycle++){

            long nowTime = System.currentTimeMillis();
            if(nowTime - ProgramStartTime> 25 * 1000){
                break;
            }

            setMecanismeDupaCon(currentCycle);
            lastCycle=currentCycle;
            Wait(250);
            //Pose2d pusConPos = drive.getPoseEstimate();

            Trajectory luatCon;
            if(currentCycle == 0) {
                luatCon = drive.trajectoryBuilder(new Pose2d(preloadX,preloadY, Math.toRadians(preloadRot)), Math.toRadians(-270))
                        .splineToSplineHeading(new Pose2d(precycleX + Vericu[currentCycle], precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        .splineTo(new Vector2d(cycle_x + Vericu[currentCycle], cycle_y + VericuY[currentCycle]), Math.toRadians(-90))
                        .build();
            } else {
                luatCon = drive.trajectoryBuilder(new Pose2d(preloadX,preloadY, Math.toRadians(preloadRot)), Math.toRadians(-270))
                        .splineToSplineHeading(new Pose2d(precycleX + Vericu[currentCycle], precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        .splineTo(new Vector2d(cycle_x + Vericu[currentCycle], cycle_y + VericuY[currentCycle]), Math.toRadians(-90))
                        .build();
            }

            /*
            TrajectorySequence luatCon = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToSplineHeading(new Pose2d(precycleX + Vericu[currentCycle],precycleY+5,Math.toRadians(precycleRot)),Math.toRadians(-90))
                    .splineTo(new Vector2d(cycle_x + Vericu[currentCycle],cycle_y + VericuY[currentCycle]),Math.toRadians(-90))
                    //,SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH)
                    //,SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

             */

            drive.followTrajectoryAsync(luatCon);

            int aa=0;
            while (drive.isBusy()){
                /*
                Pose2d currentPos = drive.getPoseEstimate();
                if(aa==0 && distance2D(pusConPos,currentPos) > distanceReset){
                    setMecanismeDupaCon(currentCycle);
                    aa=1;
                }

                 */
                if(brat.getCurrentPosition()<300){
                    clawLeft.setPosition(clawLeftOpen+0.1);
                    clawRight.setPosition(clawRightOpen);
                }

                drive.update();
            }

            clawLeft.setPosition(clawLeftClose);
            clawRight.setPosition(clawRightClose);
            Wait(100);
            ghidaj.setPosition(ghidajLift);
            clawRotate.setPosition(0.55);
            lift.setTargetPosition(liftUp);
            Wait(100);


            /*
            Trajectory pusCon = drive.trajectoryBuilder(drive.getPoseEstimate(),false)
                    .lineToLinearHeading(new Pose2d(preloadX+Vericu[currentCycle],preloadY,Math.toRadians(preloadRot)))
                    .build();

             */

            Trajectory pusCon = drive.trajectoryBuilder(drive.getPoseEstimate(),false)
                    .lineTo(new Vector2d(precycleX + Vericu[currentCycle] ,precycleY))
                    .splineToSplineHeading(new Pose2d(cycleX + Vericu[currentCycle],cycleY,Math.toRadians(cycleRot + VericuRotire[currentCycle])),Math.toRadians(cycleTg + VericuRotire[currentCycle]))
                    .build();



            setTargetPositionAfter("brat",bratUp,200);
            setTargetPositionAfter("lift",liftUp,200);
            setTargetPositionServoAfter("clawRotate",clawRotateUp,550);
            setTargetPositionServoAfter("ghidaj",ghidajUp,550);
            ghidaj.setPosition(ghidajUp);
            drive.followTrajectory(pusCon);
            drive.update();

            drive.setWeightedDrivePower(new Pose2d(1,0,0));
            drive.update();
            Wait(200);
            drive.setWeightedDrivePower(new Pose2d(0,0,0));
            drive.update();

            puneCon(currentCycle+1);

            drive.update();
        }
        if(opModeIsActive() && !isStopRequested())
        parcare(lastCycle);

    }
}
