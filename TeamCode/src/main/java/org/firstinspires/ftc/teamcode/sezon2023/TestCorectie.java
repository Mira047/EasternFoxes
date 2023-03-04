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

@Autonomous(name="Test Corectie", group="Linear Opmode")
@Config
public class TestCorectie extends LinearOpMode {

    public String[] script = new String[]{


    };

    SampleMecanumDrive drive;

    DcMotor brat,lift;
    Servo clawRotate,clawLeft,clawRight;
    int caz;

    DistanceSensor distanceSensor,distanceSensor2;
    Trajectory first;

    public static double fX = 15;
    public static double fY = 1.5;

    CameraRecognition cameraRecognition;


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
        distanceSensor2 = hardwareMap.get(DistanceSensor.class,"distanceSensor2");

        first = //drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                //.lineTo(new Vector2d(fX,fY))
                //.splineToSplineHeading(new Pose2d(preloadX, preloadY, Math.toRadians(preloadRot)),Math.toRadians(preloadTg)).build();

                drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)),false)
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

    public static double preloadX = 53;
    public static double preloadY = 3.5;
    public static double preloadRot = 40;
    public static double preloadTg = 40;

    public static double lineConstantX = 45;
    public static double lineConstantY = 0;

    public static double splineX = 55;
    public static double splineY = -20;

    public static double lineToConX=55;
    public static double lineToConY = -28;

    public static double splineRot = 90;
    public static double splineTg= -90;

    public static double sensorretard = 160;
    public static double corectieLimita = 270;
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

    public static double corectieViteza = 0.34;
    public static double corectieVitezaRot = 0.4;

    public static double clawRotate_CYCLE =0.389;
    public static int bratUp = 2090;
    public static int liftUp = 1200;
    public static int liftUp_final = 0;

    public static int[] LIFT_CYCLES = {
            780,
            615,
            445,
            320,
            0
    };

    public static double cycle_final_x = 54;
    public static double cycle_final_y = 3.2;
    public static double cycle_final_rot =50;

    Pose2d cycle_final;

    public static double cycle_x=52;
    public static double cycle_y = -22.8;
    public static double cycle_rot = 90;

    public static double precycleX = 51;
    public static double precycleY = -7;
    public static double precycleRot = 90;

    public static double cycle_x_init=52;
    public static double cycle_y_init = -22.8;
    public static double cycle_rot_init = 90;


    public static double clawLeftClose = 0.64;
    public static double clawLeftOpen = 0.36;

    public static double clawRightClose = 0.36;
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


    public class CorectieData {
        public double dist;
        public double rotImu;
    }

    public class CorectieOutput{
        public double dist;
        public int brat;
    }

    CorectieData[] corectieDataList = new CorectieData[64];

    CorectieOutput corectie(){
        double dist = 0;
        double dist2 = 0;
        long startTime = System.currentTimeMillis();
        long startTimeReal = startTime;
        double oldDirection = 0;
        double corectieDirectieTemp = corectieDirectie;
        int corectieTimeoutTemp = corectieTimeoutChange;

        int maxCorectieDataLen = corectieDataList.length;
        int corectieDataLen = 0;

        int ok=0;

        CorectieData minDist = new CorectieData();
        minDist.dist = 9999;
        String s = "";
        while (true && opModeIsActive() && !isStopRequested()){
            long t2 = System.currentTimeMillis();
            if(t2 - startTimeReal >=2500){
                CorectieOutput cc = new CorectieOutput();
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
                cc.dist= 0;
                return cc;
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
                            corectieVitezaRot * corectieDirectieTemp
                    )
            );
            drive.update();
            dist = (distanceSensor.getDistance(DistanceUnit.MM));
            //dist2 = (distanceSensor2.getDistance(DistanceUnit.MM));

            CorectieData corectieData = new CorectieData();
            corectieData.dist = dist;
            corectieData.rotImu = Math.toDegrees(drive.getRawExternalHeading());

            s += Double.toString(dist) +" / " + Double.toString(corectieData.rotImu) + "\n";

            if(dist < minDist.dist){
                minDist.dist = dist;
                minDist.rotImu = corectieData.rotImu;
            }

            if(corectieDataLen >= maxCorectieDataLen) {
                if(ok==1)
                    break;
                else{
                    corectieDataLen=0;
                }
            }

            corectieDataList[corectieDataLen] = corectieData;
            corectieDataLen++;


            if(dist < corectieLimita){
                ok=1;
                //drive.setWeightedDrivePower(new Pose2d(0,0,0));
                //break;
            }
            telemetry.addData("Data",s);
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        Wait(10000);
        telemetry.addData("Corectat cu",minDist.dist);
        telemetry.addData("Rotire imu ",minDist.rotImu);
        telemetry.addData("Rotire de acum",Math.toDegrees(drive.getRawExternalHeading()));
        telemetry.update();
        double ang = Math.toDegrees(drive.getRawExternalHeading());
        if(ang > minDist.rotImu){
            corectieDirectie = -1;
        } else {
            corectieDirectie = 1;
        }
        double thresh = 0.5; //deg
        while (Math.abs(ang-minDist.rotImu) > thresh){
            drive.setWeightedDrivePower(new Pose2d(
                    0,
                    0,
                    corectieVitezaRot * corectieDirectie
            ));
            drive.update();
            ang =  Math.toDegrees(drive.getRawExternalHeading());
            telemetry.addData("ang me ",ang);
            telemetry.addData("ang target",minDist.rotImu);
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(
                0,
                0,
                0
        ));
        drive.update();
        double cdist = 0;
        cdist = (dist + dist2) / 2;


        drive.update();

        CorectieOutput cc = new CorectieOutput();
        cc.dist = minDist.dist;
        return cc;

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
        if(dist <= sensorretard -off){
            off = 22;
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
        } else if(dist >=sensorretard-off && (dist <= 300)){
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

    private  void main(){
        clawLeft.setPosition(clawLeftClose);
        clawRight.setPosition(clawRightClose);
        corectieDirectie = 1;

        CorectieOutput ccc = corectie();
        Wait(15000);
        return;

        /*

        if(ccc.dist <1){
            return;
        }
        int x = corectie2(0,ccc);
        if(x == -1) {
            return;
        }

        brat.setTargetPosition(bratUp);
        lift.setTargetPosition(1100);
        clawRotate.setPosition(0.358); //0.362
        Wait(4000);


        lift.setTargetPosition(100);

        Wait(500);

        clawLeft.setPosition(clawLeftOpen);
        clawRight.setPosition(clawRightOpen);
        Wait(500);
        clawRotate.setPosition(0.514);
        brat.setTargetPosition(0);
        lift.setTargetPosition(0);

        drive.update();
        Wait(500);

         */

    }
}
