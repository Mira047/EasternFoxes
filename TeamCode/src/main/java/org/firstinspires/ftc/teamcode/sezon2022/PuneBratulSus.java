package org.firstinspires.ftc.teamcode.sezon2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class PuneBratulSus extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor angle = hardwareMap.get(DcMotor.class,"angle");
        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setTargetPosition(angle.getCurrentPosition());
        angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angle.setPower(1);
        angle.setTargetPosition(2200);
        waitForStart();
    }
}
