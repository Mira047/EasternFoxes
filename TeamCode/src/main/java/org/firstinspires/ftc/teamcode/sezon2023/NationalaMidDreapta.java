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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sezon2022.CameraRecognition;
import org.firstinspires.ftc.teamcode.sezon2022.PipeLineDetector;

@Autonomous(name="NationalaMidDreapta", group="Linear Opmode")
@Config
public class NationalaMidDreapta extends LinearOpMode {

    public String[] script = new String[]{


    };

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

    public static double ghidajLift=0.8;


    public static double clawRotateUp = 0.386; // 0.37;//0.2;
    public static double clawRotateMid = 0.372;
    public static double clawRotateLow = 0.325;
    public static double clawRotateLowFront = 0.5;
    public  static  double clawRotateInit = 0.506;

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
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        clawRotate.setPosition(0.56);

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

                drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        //.lineTo(new Vector2d(fX,fY))
                        .lineTo(new Vector2d(25,1))
                        .splineToSplineHeading(new Pose2d(preloadX-1.2, preloadY, Math.toRadians(preloadRot)),Math.toRadians(preloadTg))
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

    public static long ProgramStartTime=0;

    public static double preloadX = 49; //47
    public static double preloadY = 2.9;
    public static double preloadRot = 40;
    public static double preloadTg = 40;

    public static double midX = 49;
    public static double midY = 2.5;
    public static double midRot = 130;
    public static double midTg = 80;
    // -90 - 130

    public static double lineConstantX = 45;
    public static double lineConstantY = 0;

    public static double splineX = 55;
    public static double splineY = -20;

    public static double lineToConX=55;
    public static double lineToConY = -28;

    public static double splineRot = 90;
    public static double splineTg= -90;

    public static double sensorretard = 210;
    public static double corectieLimita = 310;
    public static double corectieDirectie = 1;
    public static int corectieTimeoutChange = 150;

    public static double corectieMultiBratPlus = 4;
    public static double corectieMultiBratMinus = 4.0;
    public static double corectieBaseBratMM=242.0;


    public static int CYCLES = 5;

    /*
    1 pentru stanga
    -1 pentru dreapta
     */

    public static double corectieViteza = 0.4;

    public static double clawRotate_CYCLE =0.389;
    public static int bratUp = 1965;
    public static int bratMid = 2420;
    public static int liftUp = 1100;
    public static int liftMid = 1050;
    public static int liftUp_final = 0;

    public static int[] LIFT_CYCLES = {
            820,
            675,
            445,
            310,
            0
    };

    public static double cycle_final_x = 54;
    public static double cycle_final_y = 3.2;
    public static double cycle_final_rot =50;

    Pose2d cycle_final;

    public static double cycle_x=50.9;
    public static double cycle_y = -24.3;
    public static double cycle_rot = 90;

    public static double precycleX = 50.2;
    public static double precycleY = -7;
    public static double precycleRot = 90;

    public static double cycle_x_init=50.5;
    public static double cycle_y_init = -23.4;
    public static double cycle_rot_init = 90;


    public static double clawLeftClose = 0.72;
    public static double clawLeftOpen = 0.37;

    public static double clawRightClose = 0.3;
    public static double clawRightOpen = 0.7;

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
    public static int autoPutDelay = 200;
    void puneCon(int cycle){
        brat.setTargetPosition(brat.getTargetPosition() + 550);

        int tresh = 20;
        int p = brat.getCurrentPosition();
        int initPos = p;
        int t = brat.getTargetPosition();
        while (!(p >= t - tresh && p <= t + tresh)) {
            p = brat.getCurrentPosition();
            if(Math.abs(p-initPos) > 100){
                double pos = Math.max(clawRotateUp - 0.002 * ((Math.abs(p-initPos)-100)/20),clawRotateUp-0.012);
                clawRotate.setPosition(pos);
            }
        }
        clawLeft.setPosition(clawLeftOpen);
        clawRight.setPosition(clawRightOpen);
        Wait(80);
        clawRotate.setPosition(0.522);

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

    public class CorectieOutput{
        public double dist;
        public int brat;
    }

    CorectieOutput corectie(){
        double dist = 1000;
        double dist2 = 1000;
        long startTime = System.currentTimeMillis();
        long startTimeReal = startTime;
        double oldDirection = 0;
        double corectieDirectieTemp = corectieDirectie;
        int corectieTimeoutTemp = corectieTimeoutChange;
        while (true && opModeIsActive() && !isStopRequested()){
            long t2 = System.currentTimeMillis();
            if(t2 - startTimeReal >=2500){
                CorectieOutput c = new CorectieOutput();
                c.dist = -1;
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );
                telemetry.addData("Distanta de fail",dist);
                telemetry.update();
                drive.update();
                return c;
            }
            if(t2 - startTime > corectieTimeoutTemp + corectieTimeoutTemp * Math.abs(oldDirection)){
                startTime = t2;
                oldDirection = corectieDirectieTemp;
                corectieDirectieTemp = -corectieDirectieTemp;
                corectieTimeoutTemp += 90;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            corectieViteza * corectieDirectieTemp
                    )
            );
            drive.update();
            dist = distanceSensor.getDistance(DistanceUnit.MM);
            dist2 = (distanceSensor2.getDistance(DistanceUnit.MM));
            if(dist < corectieLimita && dist2 < corectieLimita && Math.abs(dist-dist2) <= 23){
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                break;
            }
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));

        CorectieOutput c = new CorectieOutput();
        c.brat = bratUp;
        c.dist = dist ;
        return c;
    }

    int corectie2(int cycle,CorectieOutput c){
        double dist = 1000;
        dist = c.dist;//distanceSensor.getDistance(DistanceUnit.MM);
        double off =0;
        if(cycle >=3){


            off = 0;
            // return 0;
        } else if(cycle>=2){
            off = 0;
        }
        long startTime = System.currentTimeMillis();
        long startTimeReal = startTime;
        drive.update();
        Pose2d oldPose = drive.getPoseEstimate();
        if(sensorretard>250) {
            off = 0;
        }
        else {
            off = 0;
        }
        if(dist <= sensorretard -off){
            if(sensorretard<250){
                off = 0;
            }
            while (dist <=sensorretard -off) {

                long t2 = System.currentTimeMillis();
                if(t2 - startTimeReal >=2000){

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0,
                                    0
                            )
                    );
                    telemetry.addData("Distanta de fail",dist);
                    telemetry.update();
                    drive.update();
                    return -1;
                }

                drive.update();
                Pose2d current_pos = drive.getPoseEstimate();
                double x = (current_pos.getX()-oldPose.getX());
                double y = (current_pos.getY() - oldPose.getY());
                double m = Math.sqrt(x*x + y * y ) * 2.54 * 10;
                /*
                telemetry.addData("X OLD",oldPose.getX());
                telemetry.addData("Y OLD",oldPose.getY());
                telemetry.addData("X NOW",current_pos.getX());
                telemetry.addData("X NOW",current_pos.getY());
                telemetry.addData("Am mers",m);
                telemetry.update();

                 */
                dist = c.dist + m;
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -corectieViteza*1,
                                0,
                                0
                        )
                );
                //dist = distanceSensor.getDistance(DistanceUnit.MM);
            }
        } else if(dist >=sensorretard-off){
            if(sensorretard <250){
                off = 0;
            }
            while (dist >=sensorretard-off) {
                long t2 = System.currentTimeMillis();
                if(t2 - startTimeReal >=2000){

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0,
                                    0
                            )
                    );
                    telemetry.addData("Distanta de fail",dist);
                    telemetry.update();
                    drive.update();
                    return -1;
                }
                drive.update();
                Pose2d current_pos = drive.getPoseEstimate();
                double x = (current_pos.getX()-oldPose.getX());
                double y = (current_pos.getY() - oldPose.getY());
                double m = Math.sqrt(x*x + y * y ) * 2.54 * 10;
                /*
                telemetry.addData("X OLD",oldPose.getX());
                telemetry.addData("Y OLD",oldPose.getY());
                telemetry.addData("X NOW",current_pos.getX());
                telemetry.addData("X NOW",current_pos.getY());
                telemetry.addData("Am mers",m);
                telemetry.update();
                 */
                dist = c.dist - m;
                drive.setWeightedDrivePower(
                        new Pose2d(
                                corectieViteza*1,
                                0,
                                0
                        )
                );
                //dist = distanceSensor.getDistance(DistanceUnit.MM);
            }
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        telemetry.addData("Corectat cu mers",dist);
        telemetry.update();
        return 0;
    }

    double distance2D(Pose2d p1,Pose2d p2){
        double x = p1.getX()-p2.getX();
        double y= p1.getY() - p2.getY();
        return Math.sqrt(x*x+y*y) * 2.54;
    }

    void parcare(int cycle){
        brat.setTargetPosition(0);
        lift.setTargetPosition(0);
        clawRotate.setPosition(0.55);
        if(caz == 1)
            Wait(150);
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);

        if(!opModeIsActive() || isStopRequested())
            return;

        if(caz == 1){
            Trajectory parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(50.5 + cycle * 0.5,25.5,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(parcare);
            drive.update();
            Pose2d cPos = drive.getPoseEstimate();
            if(distance2D(cPos,new Pose2d(51,27.5)) > 30){
                parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(50.5 + cycle * 0.5,25.5,Math.toRadians(0)))
                        .build();
                drive.followTrajectory(parcare);
                drive.update();
            }
        } else if(caz == 2){
            Trajectory parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                    //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(48,0,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(parcare);
        } if(caz == 3){

            Trajectory parcare = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                    .splineToSplineHeading(new Pose2d(precycleX + cycle * 0.58,precycleY,Math.toRadians(precycleRot)),Math.toRadians(-90))
                    //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                    .splineTo(new Vector2d(cycle_x + cycle * 0.5,cycle_y+2),Math.toRadians(-90))
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

    private  void main(){
        ProgramStartTime = System.currentTimeMillis();
        sensorretard = 232;
        int cycle=0;
        cycle_x = cycle_x_init;
        cycle_y = cycle_y_init;
        cycle_rot = cycle_rot_init;
        cycle_final = new Pose2d(cycle_final_x,cycle_final_y,Math.toRadians(cycle_final_rot));
        corectieDirectie = 1;


        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        clawRotate.setPosition(0.55);
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

        // Pose2d cPos = drive.getPoseEstimate();
        //Pose2d conNew = new Pose2d(preloadX-cPos.getX() + cycle_x,preloadY - cPos.getY() + cycle_y,Math.toRadians(90));


        //clawLeft.setPosition(0.57);
        //clawRight.setPosition(0.44);
        //clawRotate.setPosition(0.56);

        sensorretard = 274;
        corectieLimita=300;

        for(int i=0;i<CYCLES && opModeIsActive() && !isStopRequested();i++){
            long nowTime = System.currentTimeMillis();
            if(nowTime-ProgramStartTime> 25000){
                break;
            }
            if(i<4)
                brat.setPower(1);
            else
                brat.setPower(1);
            brat.setTargetPosition(0);
            lift.setTargetPosition(LIFT_CYCLES[i]);
            Wait(100);
            Trajectory traj;
            /*
            if(i<4){
            traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(cycle_x,cycle_y,Math.toRadians(cycle_rot)))
                    .build();
            } else{
                traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(cycle_x,cycle_y+0.95,Math.toRadians(cycle_rot)))
                        .build();
            }*/
            drive.update();
            if(i==0) {
                traj = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(-270))
                        .splineToSplineHeading(new Pose2d(precycleX + i * 0.62, precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        .splineTo(new Vector2d(cycle_x + i * 0.1, cycle_y), Math.toRadians(-90),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
                        //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                        //.splineToLinearHeading()
                        .build();
            }
            else if(i>=3){
                traj = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                        .splineToSplineHeading(new Pose2d(precycleX + i * 0.52, precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        .splineTo(new Vector2d(cycle_x + i * 0.1, cycle_y + (i-2) * 0.1), Math.toRadians(-90),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 20))
                        //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                        //.splineToLinearHeading()
                        .build();
            }
            else{
                traj = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                        .splineToSplineHeading(new Pose2d(precycleX + i * 0.62, precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        .splineTo(new Vector2d(cycle_x + i * 0.1, cycle_y-0.1*i), Math.toRadians(-90),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 20))
                        //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                        //.splineToLinearHeading()
                        .build();
            }
            Pose2d targetCyclePos = new Pose2d(cycle_x + i * 0.1,cycle_y,Math.toRadians(90));
            drive.followTrajectoryAsync(traj);
            int aaa=0;
            while (drive.isBusy()) {
                if(brat.getCurrentPosition()<450){
                    clawLeft.setPosition(clawLeftOpen);
                    clawRight.setPosition(clawRightOpen);
                } else if(brat.getCurrentPosition() < 2100 && aaa==0){
                    aaa=1;
                    clawLeft.setPosition(clawLeftClose);
                    clawRight.setPosition(clawRightClose);
                    setTargetPositionServoAfter("clawRotate",0.524,130);
                    //clawRotate.setPosition(0.522);
                }
                drive.update();
            }
            clawLeft.setPosition(clawLeftClose);
            clawRight.setPosition(clawRightClose);
            long cTime = System.currentTimeMillis();
            traj = drive.trajectoryBuilder(targetCyclePos,false)
                    .lineTo(new Vector2d(precycleX + i * 0.1 ,precycleY))
                    .splineToSplineHeading(new Pose2d(midX + i * 0.45,midY - 0.12 * i,Math.toRadians(midRot)),Math.toRadians(midTg))
                    .build();
            long dTime = System.currentTimeMillis();
            if(dTime-cTime<170){
                Wait((int)(170-(dTime-cTime)));
            }
            clawRotate.setPosition(0.55);
            lift.setTargetPosition(liftUp);
            Wait(100);
            ghidaj.setPosition(ghidajLift);
            brat.setPower(1);
            setTargetPositionAfter("brat",2000,650);
            setTargetPositionServoAfter("ghidaj",ghidajMid,1000);
            setTargetPositionAfter("lift",liftMid,200);
            setTargetPositionServoAfter("clawRotate",0.359,1000);
            drive.update();
            drive.followTrajectory(traj);
            drive.update();

            ghidaj.setPosition(ghidajMid);

            CorectieOutput cc = corectie();
            if(cc.dist<0){
                break;
            }
            int xx = corectie2(i,cc);
            if(xx==-1)
                break;

            cycle++;


            brat.setTargetPosition(bratMid);
            clawRotate.setPosition(0.359);
            if(opModeIsActive() == false || isStopRequested())
                break;
            Pose2d currentPose = drive.getPoseEstimate();
            //if(distance2D(currentPose,new Pose2d(cycle_final_x,cycle_final_y,0)) < 30)
            // cycle_final = currentPose;

            Wait(150);
            lift.setTargetPosition(liftUp_final);
            Wait(250);
            clawLeft.setPosition(clawLeftOpen);
            clawRight.setPosition(clawRightOpen-0.15);
            Wait(140);
        }

        parcare(cycle);

    }
}
