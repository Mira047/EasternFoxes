package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sezon2022.CameraRecognition;
import org.firstinspires.ftc.teamcode.sezon2022.HardwareTesterInterpreter;
import org.firstinspires.ftc.teamcode.sezon2022.PipeLineDetector;

@Autonomous(name="Auto2023 Dreapta", group="Linear Opmode")
@Config
public class Auto2023Dreapta extends LinearOpMode {

    public String[] script = new String[]{
            "servo clawRotate 0.55",
            "servo clawLeft 0.58",
            "servo clawRight 0.42",
            "go posh 27.25 0 33",
            "motor lift 0 reset_position",
            "motor lift 1 power",
            "motor lift 760 position",
            "motor brat 0 reset_position",
            "motor brat 0.9 power",
            "motor brat -2250 position",
            "servo clawRotate 0.352",
            "motor brat wait position 50",
            "motor lift 50 position",
            "motor lift wait position 20",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "wait 100",
            "servo clawLeft 0.49",
            "servo clawRight 0.475",
            "servo clawRotate 0.565",
            "motor lift 0 position",
            "motor brat 0 position",
           // "wait 100",
            "go posh 55 -2 90",
            "motor lift 690 position",
            "servo clawRotate 0.519",
            "wait 100",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "go posh 55.5 -27.1 92",
            "wait 100",
            "servo clawLeft 0.55",
            "servo clawRight 0.45",
            "wait 200",
            "motor lift 1400 position",
            "wait 100",
            "servo clawRight 0.44",
            "motor lift wait position 80",
            "servo clawRotate 0.555",
            "wait 400",
            "motor lift 1300 position",
            "motor brat 0.6 power",
            "go posh 52 -18 81",
            "servo clawRotate 0.38",
            "motor brat -2050 position",
            "go posh 53.2 4.5 28",
            "motor lift 200 position",
            "motor lift wait position 40",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "wait 200",
            "servo clawLeft 0.55",
            "servo clawRight 0.44",
            "servo clawRotate 0.51",
            "motor brat 0.48 power",
            "motor brat 0 position",
            "motor lift 560 position",
            "servo clawRotate 0.513",
            "motor lift wait position 20",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "go posh 55.5 -27.2 92.5",
            "motor brat 0.6 power",
            "servo clawLeft 0.55",
            "servo clawRight 0.44",
            "wait 300",
            "motor lift 1300 position",
            "wait 100",
            "motor lift wait position 80",
            "servo clawRotate 0.555",
            "wait 300",
            "motor lift 1300 position",
            "motor brat 0.6 power",
            "go posh 52 -18 81",
            "servo clawRotate 0.38",
            "motor brat -2050 position",
            "go posh 53 4.1 29",
            "motor lift 200 position",
            "motor lift wait position 30",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "wait 200",
            "servo clawLeft 0.55",
            "servo clawRight 0.44",
            "servo clawRotate 0.51",
            "motor brat 0 position",
            "motor lift 425 position",
            "servo clawRotate 0.512",
            "motor lift wait position 30",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "go posh 55.5 -27.5 94",
            "servo clawLeft 0.555",
            "servo clawRight 0.425",
            "wait 200",
            "motor lift 1300 position",
            "servo clawRight 0.44",
            "motor lift wait position 80",
            "servo clawRotate 0.555",
            "wait 120",
            "motor lift 1300 position",
            "motor brat 0.6 power",
            "go posh 52 -18 81",
            "servo clawRotate 0.39",
            "motor brat -2050 position",
            "go posh 52.5 4 29.3",
            "wait 199",
            "motor lift 200 position",
            "motor lift wait position 40",
            "servo clawLeft 0.3",
            "servo clawRight 0.7",
            "wait 100",
            "motor brat 0 position",
            "motor lift 0 position",
            "servo clawRotate 0.57",
            "servo clawLeft 0.55",
            "servo clawRight 0.44",
            "motor brat wait position 500"
    };
    SampleMecanumDrive drive;
    CameraRecognition cameraRecognition;
    int caz;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        HardwareTesterInterpreter.initHWI(this,hardwareMap,telemetry,drive);

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


    private  void main(){
        for(int i=0;i<script.length;i++){
            if(isStopRequested())
            {
                break;
            }
            HardwareTesterInterpreter.interpretCommand(script[i]);
        }
        if(isStopRequested())
            return;
        if(caz == 1){
            HardwareTesterInterpreter.interpretCommand("go posh 52 21 90");
        } else if(caz == 2){
            HardwareTesterInterpreter.interpretCommand("go posh 48 0 0");
        } if(caz == 3){
                HardwareTesterInterpreter.interpretCommand("go posh 48 -28 0");
        }
    }
}
