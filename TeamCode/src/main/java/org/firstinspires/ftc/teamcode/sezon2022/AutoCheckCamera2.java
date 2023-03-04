package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoCheckCamera Blue", group="Linear Opmode")
@Config
public class AutoCheckCamera2 extends  LinearOpMode{

    public static int tresh = 140;
    CameraRecognition cameraRecognition;

    @Override
    public void runOpMode() throws InterruptedException {
        cameraRecognition =  new CameraRecognition(hardwareMap,telemetry,"blue");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);
        waitForStart();
        while (opModeIsActive()) {
            cameraRecognition.tresh = tresh;
            telemetry.addData("v1",cameraRecognition.v1);
            telemetry.update();
            //telemetry.addData("case",cameraRecognition.getCase());
            //telemetry.update();
        }
        cameraRecognition.stop();
    }
}
