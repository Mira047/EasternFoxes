package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Test Senzor", group="Linear Opmode")
@Config
public class TestSenzor extends LinearOpMode {


    DistanceSensor distanceSensor,distanceSensor2;

    @Override
    public void runOpMode() throws InterruptedException {

        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class,"distanceSensor2");

        waitForStart ();

        while (opModeIsActive()) {
            telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("distance2",distanceSensor2.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }



    void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (Exception ex){

        }
    }

}
