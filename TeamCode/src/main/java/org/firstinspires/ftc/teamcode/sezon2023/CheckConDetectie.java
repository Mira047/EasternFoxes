package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="CheckConDetectie", group="Linear Opmode")
@Config
public class CheckConDetectie extends  LinearOpMode{

    public static int tresh = 140;
    DetectieConCamera cameraRecognition;
    public static int rezX = 640;
    public static int rezY = 480;

    SampleMecanumDrive drive;

    double dirLeft = -1;
    double dirRight = 1;
    double rotSpeed = 0.28;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        cameraRecognition =  new QRDetect(hardwareMap,telemetry);
        cameraRecognition.rezX = rezX;
        cameraRecognition.rezY = rezY;
        cameraRecognition.initCamera();
        cameraRecognition.start();

         */
        cameraRecognition = new DetectieConCamera(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);

        drive = new SampleMecanumDrive(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()){
            int c =  cameraRecognition.getCase();
            telemetry.addData("detected",c);
            telemetry.update();
            Pose2d p;
            if(c==1){
                p = new Pose2d(0,0,rotSpeed * dirLeft);
            } else if(c==3){
                p=new Pose2d(0,0,rotSpeed*dirRight);
            } else {
                p = new Pose2d(0,0,0);
            }
            drive.setWeightedDrivePower(p);
            drive.update();
        }
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Caz", cameraRecognition.getCase());
            telemetry.addLine("Init Complete");
            telemetry.update();

            //telemetry.addData("case",cameraRecognition.getCase());
            //telemetry.update();
        }
        cameraRecognition.stop();
    }
}
