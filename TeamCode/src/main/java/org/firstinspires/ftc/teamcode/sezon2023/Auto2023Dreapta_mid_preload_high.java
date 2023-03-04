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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sezon2022.CameraRecognition;
import org.firstinspires.ftc.teamcode.sezon2022.PipeLineDetector;

@Autonomous(name="Auto2023 Dreapta Mid Preload High", group="Linear Opmode")
@Config
public class Auto2023Dreapta_mid_preload_high extends LinearOpMode {

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

    public static double ghidajUp = 0.55;
    public static double ghidajMid = 0.4;
    public static double ghidajLow = 0.45;
    public static double ghidajIdle = 0.65; // 0.45


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
                        .lineTo(new Vector2d(31,1))
                        .splineToSplineHeading(new Pose2d(preloadX, preloadY, Math.toRadians(preloadRot)),Math.toRadians(preloadTg))
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

    public static double preloadX = 52;
    public static double preloadY = 3;
    public static double preloadRot = 38;
    public static double preloadTg = 38;

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
    public static int bratUp = 2110;
    public static int bratMid = 2385;
    public static int liftUp = 1400;
    public static int liftMid = 1050;
    public static int liftUp_final = 0;

    public static int[] LIFT_CYCLES = {
            820,
            655,
            425,
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

    public static double precycleX = 51;
    public static double precycleY = -7;
    public static double precycleRot = 90;

    public static double cycle_x_init=50.5;
    public static double cycle_y_init = -23.8;
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
                    .lineToLinearHeading(new Pose2d(51 + cycle * 0.5,24,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(parcare);
            drive.update();
            Pose2d cPos = drive.getPoseEstimate();
            if(distance2D(cPos,new Pose2d(51,22)) > 30){
                parcare = drive.trajectoryBuilder(drive.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(51,22,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(51 + cycle * 0.5,24,Math.toRadians(0)))
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
        sensorretard = 232;
        int cycle=0;
        cycle_x = cycle_x_init;
        cycle_y = cycle_y_init;
        cycle_rot = cycle_rot_init;
        cycle_final = new Pose2d(cycle_final_x,cycle_final_y,Math.toRadians(cycle_final_rot));
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        corectieDirectie = 1;
        lift.setTargetPosition(1400);
        //clawRotate.setPosition(0.45);

        setTargetPositionAfter("brat",1700,550);
        setTargetPositionServoAfter("clawRotate",0.362,550);
        drive.followTrajectory(first);



        CorectieOutput ccc = corectie();
        if(ccc.dist <1){
            parcare(0);
            return;
        }
        int x = corectie2(0,ccc);
        if(x == -1) {
            parcare(0);
            return;
        }

        //Wait(5000);
        brat.setTargetPosition(bratUp+150);
        lift.setTargetPosition(1400);
        clawRotate.setPosition(0.362);
        Wait(200);


        lift.setTargetPosition(200);

        Wait(300);

        clawLeft.setPosition(clawLeftOpen);
        clawRight.setPosition(clawRightOpen);
        Wait(70);

        drive.update();

        // Pose2d cPos = drive.getPoseEstimate();
        //Pose2d conNew = new Pose2d(preloadX-cPos.getX() + cycle_x,preloadY - cPos.getY() + cycle_y,Math.toRadians(90));


        //clawLeft.setPosition(0.57);
        //clawRight.setPosition(0.44);
        //clawRotate.setPosition(0.56);

        sensorretard = 285  ;
        corectieLimita=340;

        for(int i=0;i<CYCLES && opModeIsActive() && !isStopRequested();i++){
            if(i<4)
                brat.setPower(1);
            else
                brat.setPower(1);
            brat.setTargetPosition(0);
            lift.setTargetPosition(LIFT_CYCLES[i]);
            clawLeft.setPosition(clawLeftClose);
            clawRight.setPosition(clawRightClose);
            Wait(50);
            clawRotate.setPosition(0.518);
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
            double xxx = cycle_y;
            if(i==4){
                cycle_y = xxx + 1;
            }
            if(i==0) {
                traj = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(-130))
                        .splineToSplineHeading(new Pose2d(precycleX + i * 0.62, precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        .splineTo(new Vector2d(cycle_x + i * 0.2, cycle_y), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                        //.splineToLinearHeading()
                        .build();
            }
            else if(i>=3){
                traj = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                        .splineToSplineHeading(new Pose2d(precycleX + i * 0.52, precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        .splineTo(new Vector2d(cycle_x + i * 0.3, cycle_y + (i-2) * 1), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                        //.splineToLinearHeading()
                        .build();
            }
            else{
                traj = drive.trajectoryBuilder(drive.getPoseEstimate(),true)
                        .splineToSplineHeading(new Pose2d(precycleX + i * 0.62, precycleY, Math.toRadians(precycleRot)), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(34+3,-63+50,Math.toRadians(180)))
                        .splineTo(new Vector2d(cycle_x + i * 0.3, cycle_y), Math.toRadians(-90))
                        //.lineToLinearHeading(new Pose2d(precycleX,precycleY,Math.toRadians(precycleRot)))
                        //.splineToLinearHeading()
                        .build();
            }
            cycle_y = xxx;
            Pose2d targetCyclePos = new Pose2d(cycle_x + i * 0.1,cycle_y,Math.toRadians(90));
            drive.followTrajectoryAsync(traj);
            traj = drive.trajectoryBuilder(targetCyclePos,false)
                    .lineTo(new Vector2d(precycleX + i * 0.1 ,precycleY))
                    .splineToSplineHeading(new Pose2d(midX + i * 0.45,midY - 0.12 * i,Math.toRadians(midRot)),Math.toRadians(midTg))
                    .build();
            while (drive.isBusy()) {
                if(brat.getCurrentPosition()<200){
                    clawLeft.setPosition(clawLeftOpen);
                    clawRight.setPosition(clawRightOpen);
                }
                Wait(1);
                drive.update();
            }

            clawLeft.setPosition(clawLeftClose);
            clawRight.setPosition(clawRightClose);
            Wait(160);
            clawRotate.setPosition(0.55);
            lift.setTargetPosition(liftUp);
            Wait(100);
            brat.setPower(1);
            setTargetPositionAfter("brat",2000,750);
            setTargetPositionAfter("lift",liftMid,200);
            setTargetPositionServoAfter("clawRotate",0.345,750);
            //new Pose2d(cycle_x,cycle_y,Math.toRadians(cycle_rot))
            drive.update();


            //.lineToSplineHeading(new Pose2d(cycle_final_x,cycle_final_y,Math.toRadians(cycle_final_rot)))

            //.splineToConstantHeading(new Vector2d(cycle_x, cycle_y + 10),Math.toRadians(90))
            //.splineToSplineHeading(new Pose2d(cycle_final_x,cycle_final_y,Math.toRadians(cycle_final_rot)),Math.toRadians(90))
            // .lineToLinearHeading(cycle_final)
            drive.followTrajectory(traj);
            drive.update();

            CorectieOutput cc = corectie();
            if(cc.dist<0){
                break;
            }
            int xx = corectie2(i,cc);
            if(xx==-1)
                break;

            cycle++;

            ghidaj.setPosition(ghidajMid);
            brat.setTargetPosition(bratMid);
            clawRotate.setPosition(0.345);
            if(opModeIsActive() == false || isStopRequested())
                break;
            Pose2d currentPose = drive.getPoseEstimate();
            //if(distance2D(currentPose,new Pose2d(cycle_final_x,cycle_final_y,0)) < 30)
            // cycle_final = currentPose;

            Wait(120);
            lift.setTargetPosition(liftUp_final);
            Wait(150);
            clawLeft.setPosition(clawLeftOpen);
            clawRight.setPosition(clawRightOpen);
            Wait(50);
            if(i == 4)
                Wait(70);
            Wait(50);
        }

        parcare(cycle);

    }
}
