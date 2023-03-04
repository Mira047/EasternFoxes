package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoCheckCamera", group="Linear Opmode")
@Config
public class AutoCheckCamera extends  LinearOpMode{

    public static int tresh = 140;
    CameraRecognition cameraRecognition;
    public static int rezX = 640;
    public static int rezY = 480;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        cameraRecognition =  new QRDetect(hardwareMap,telemetry);
        cameraRecognition.rezX = rezX;
        cameraRecognition.rezY = rezY;
        cameraRecognition.initCamera();
        cameraRecognition.start();

         */
        cameraRecognition = new CameraRecognition(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);

        while(!opModeIsActive() && !isStopRequested()){
            int caz=0;
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
