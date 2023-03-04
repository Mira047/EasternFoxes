package org.firstinspires.ftc.teamcode.sezon2022;

import android.hardware.camera2.params.BlackLevelPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(name="Auto2022Mondiale", group="Linear Opmode")
@Config
public class Auto2022Mondiale extends LinearOpMode {

    public static double clawOpenIntake = 0.45;
    public static double extenderPower = 1;
    public static double anglePower = 1;
    public static int angleUp = -425;
    public static int angleMid = -650;
    public static int angleLow = -900;
    public static int extenderUp = -1600;
    public static int extenderMid = -1100;
    public static int extenderLow = -1100;
    public static double turretRotationUp = 120;
    public static double turretRotationMid = 120;
    public static double turretRotationLow = 120;

    CameraRecognition cameraRecognition;
    SampleMecanumDrive drive;
    Turret turret;

    String scriptLow = "c2Vydm8gY2xhdyAwLjY1CndhaXQgMjAwCm1vdG9yIGV4dGVuZGVyIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgMSBwb3dlcgptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgYW5nbGUgMSBwb3dlcgptb3RvciBhbmdsZSAtOTEwIHBvc2l0aW9uCnR1cnJldCAxMjIgd2FpdCAwLjUKbW90b3IgYW5nbGUgd2FpdCBwb3NpdGlvbiA0MAptb3RvciBleHRlbmRlciAtMTA4MCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDUwCnNlcnZvIGNsYXcgMC4zNAp3YWl0IDMwMAptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIC03MDAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiA1MAp0dXJyZXQgMC44IHBvd2VyCnR1cnJldCAwIHdhaXQgMjAKdHVycmV0IDAuNCBwb3dlcgptb3RvciBhbmdsZSAtMTQwMCBwb3NpdGlvbgpzZXJ2byBpbnRha2Vfc2Vydm8xIDEKc2Vydm8gaW50YWtlX3NlcnZvMiAwCnNlcnZvIGNsYXcgMC40NQpnbyBwb3NoIDI1LjggLTAuOCAwCnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41CndhaXQgMTAwCnNlcnZvIGNsYXcgMC42Mgp3YWl0IDMwMAptb3RvciBhbmdsZSAtNDUwIHBvc2l0aW9uCnR1cnJldCAxMjIKd2FpdCAyNTAKZ28gcG9zaCAwIC0wLjggMAptb3RvciBhbmdsZSB3YWl0IHBvc2l0aW9uIDQwCnR1cnJldCAxMjAgd2FpdCAwLjUKbW90b3IgZXh0ZW5kZXIgLTE0MDAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiAyMAp3YWl0IDEwMApzZXJ2byBjbGF3IDAuMzUKc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUKd2FpdCAyMDAKbW90b3IgZXh0ZW5kZXIgMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDEwMAp0dXJyZXQgMC44IHBvd2VyCnR1cnJldCAwIHdhaXQgMC4yCnR1cnJldCAwLjQgcG93ZXIKbW90b3IgYW5nbGUgLTE0MDAgcG9zaXRpb24Kc2Vydm8gaW50YWtlX3NlcnZvMSAxCnNlcnZvIGludGFrZV9zZXJ2bzIgMApzZXJ2byBjbGF3IDAuNDUKZ28gcG9zaCAzMCAtMC44IDAKc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUKd2FpdCAxMDAKc2Vydm8gY2xhdyAwLjYyCndhaXQgMzAwCm1vdG9yIGFuZ2xlIC00NTAgcG9zaXRpb24KdHVycmV0IDEyMgpnbyBwb3NoIDAgLTAuNSAwCm1vdG9yIGFuZ2xlIHdhaXQgcG9zaXRpb24gNDAKdHVycmV0IDEyMiB3YWl0IDAuNQptb3RvciBleHRlbmRlciAtMTQwMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDIwCndhaXQgMTAwCnNlcnZvIGNsYXcgMC4zNQpzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQp3YWl0IDIwMAptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMTAwCnR1cnJldCAwLjggcG93ZXIKdHVycmV0IDAgd2FpdCAwLjIKdHVycmV0IDAuNCBwb3dlcgptb3RvciBhbmdsZSAtMTQwMCBwb3NpdGlvbgpzZXJ2byBpbnRha2Vfc2Vydm8xIDEKc2Vydm8gaW50YWtlX3NlcnZvMiAwCnNlcnZvIGNsYXcgMC40NQpnbyBwb3NoIDIyIC0xIDAKZ28gcG9zaCAzNCAzLjYgMTkKc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUKd2FpdCAxMDAKc2Vydm8gY2xhdyAwLjYyCndhaXQgMzAwCm1vdG9yIGFuZ2xlIC00NTAgcG9zaXRpb24KdHVycmV0IDEyMgpnbyBwb3NoIDI4IC0xIDAKZ28gcG9zaCAwIC0xIDAKbW90b3IgYW5nbGUgd2FpdCBwb3NpdGlvbiA0MAp0dXJyZXQgMTIyIHdhaXQgMC41Cm1vdG9yIGV4dGVuZGVyIC0xNDAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMjAKd2FpdCAxMDAKc2Vydm8gY2xhdyAwLjM1CnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41CndhaXQgMjAwCm1vdG9yIGV4dGVuZGVyIDAgcG9zaXRpb24Kc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUgCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMTAwCnR1cnJldCAwCmdvIHBvc2ggMzAgLTAuOCAwCm1vdG9yIGFuZ2xlIC0xNDAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIHdhaXQgcG9zaXRpb24=";
    String scriptMid = "c2Vydm8gY2xhdyAwLjY1CndhaXQgMjAwCm1vdG9yIGV4dGVuZGVyIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgMSBwb3dlcgptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgYW5nbGUgMSBwb3dlcgptb3RvciBhbmdsZSAtNzI1IHBvc2l0aW9uIAp0dXJyZXQgMTIyIHdhaXQgMC41Cm1vdG9yIGFuZ2xlIHdhaXQgcG9zaXRpb24gNDAKbW90b3IgZXh0ZW5kZXIgLTExMzAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiA1MApzZXJ2byBjbGF3IDAuMzYKd2FpdCA0MDAKbW90b3IgZXh0ZW5kZXIgMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDUwCnR1cnJldCAwLjggcG93ZXIKdHVycmV0IDAgd2FpdCAyMAp0dXJyZXQgMC40IHBvd2VyCm1vdG9yIGFuZ2xlIC0xNDAwIHBvc2l0aW9uCnNlcnZvIGludGFrZV9zZXJ2bzEgMQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAKc2Vydm8gY2xhdyAwLjQ1CmdvIHBvc2ggMjUuOCAtMC44IDAKc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUKd2FpdCAxMDAKc2Vydm8gY2xhdyAwLjYyCndhaXQgMjAwCm1vdG9yIGFuZ2xlIC00NTAgcG9zaXRpb24KdHVycmV0IDEyMgp3YWl0IDI1MApnbyBwb3NoIDAgLTAuOCAwCm1vdG9yIGFuZ2xlIHdhaXQgcG9zaXRpb24gNDAKdHVycmV0IDEyMCB3YWl0IDAuNQptb3RvciBleHRlbmRlciAtMTQwMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDIwCndhaXQgMTAwCnNlcnZvIGNsYXcgMC4zNQpzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQp3YWl0IDIwMAptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMTAwCnR1cnJldCAwLjggcG93ZXIKdHVycmV0IDAgd2FpdCAwLjIKdHVycmV0IDAuNCBwb3dlcgptb3RvciBhbmdsZSAtMTQwMCBwb3NpdGlvbgpzZXJ2byBpbnRha2Vfc2Vydm8xIDEKc2Vydm8gaW50YWtlX3NlcnZvMiAwCnNlcnZvIGNsYXcgMC40NQpnbyBwb3NoIDMwIC0wLjggMApzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQp3YWl0IDEwMApzZXJ2byBjbGF3IDAuNjIKd2FpdCAyMDAKbW90b3IgYW5nbGUgLTQ1MCBwb3NpdGlvbgp0dXJyZXQgMTIyCmdvIHBvc2ggMCAtMC41IDAKbW90b3IgYW5nbGUgd2FpdCBwb3NpdGlvbiA0MAp0dXJyZXQgMTIyIHdhaXQgMC41Cm1vdG9yIGV4dGVuZGVyIC0xNDAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMjAKd2FpdCAxMDAKc2Vydm8gY2xhdyAwLjM1CnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41CndhaXQgMjAwCm1vdG9yIGV4dGVuZGVyIDAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiAxMDAKdHVycmV0IDAuOCBwb3dlcgp0dXJyZXQgMCB3YWl0IDAuMgp0dXJyZXQgMC40IHBvd2VyCm1vdG9yIGFuZ2xlIC0xNDAwIHBvc2l0aW9uCnNlcnZvIGludGFrZV9zZXJ2bzEgMQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAKc2Vydm8gY2xhdyAwLjQ1CmdvIHBvc2ggMjIgLTEgMApnbyBwb3NoIDM0IDMuNiAxOQpzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQp3YWl0IDEwMApzZXJ2byBjbGF3IDAuNjIKd2FpdCAyMDAKbW90b3IgYW5nbGUgLTQ1MCBwb3NpdGlvbgp0dXJyZXQgMTIyCmdvIHBvc2ggMjggLTEgMApnbyBwb3NoIDAgLTEgMAptb3RvciBhbmdsZSB3YWl0IHBvc2l0aW9uIDQwCnR1cnJldCAxMjIgd2FpdCAwLjUKbW90b3IgZXh0ZW5kZXIgLTE0MDAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiAyMAp3YWl0IDEwMApzZXJ2byBjbGF3IDAuMzUKc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUKd2FpdCAyMDAKbW90b3IgZXh0ZW5kZXIgMCBwb3NpdGlvbgpzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDEwMAp0dXJyZXQgMApnbyBwb3NoIDMwIC0wLjggMAptb3RvciBhbmdsZSAtMTQwMCBwb3NpdGlvbgptb3RvciBhbmdsZSB3YWl0IHBvc2l0aW9u";
    String scriptHigh = "c2Vydm8gY2xhdyAwLjY1CndhaXQgMjAwCm1vdG9yIGV4dGVuZGVyIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgMSBwb3dlcgptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIDAgcmVzZXRfcG9zaXRpb24KbW90b3IgYW5nbGUgMSBwb3dlcgptb3RvciBhbmdsZSAtNDUwIHBvc2l0aW9uCnR1cnJldCAxMjIgd2FpdCAwLjUKbW90b3IgYW5nbGUgd2FpdCBwb3NpdGlvbiA0MAptb3RvciBleHRlbmRlciAtMTQwMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDUwCnNlcnZvIGNsYXcgMC4zNAp3YWl0IDQwMAptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gNTAKdHVycmV0IDAuOCBwb3dlcgp0dXJyZXQgMCB3YWl0IDIwCnR1cnJldCAwLjQgcG93ZXIKbW90b3IgYW5nbGUgLTE0MDAgcG9zaXRpb24Kc2Vydm8gaW50YWtlX3NlcnZvMSAxCnNlcnZvIGludGFrZV9zZXJ2bzIgMApzZXJ2byBjbGF3IDAuNDUKZ28gcG9zaCAyNS44IC0wLjggMApzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQp3YWl0IDEwMApzZXJ2byBjbGF3IDAuNjIKd2FpdCAzMDAKbW90b3IgYW5nbGUgLTQ1MCBwb3NpdGlvbgp0dXJyZXQgMTIyCndhaXQgMjUwCmdvIHBvc2ggMCAtMC44IDAKbW90b3IgYW5nbGUgd2FpdCBwb3NpdGlvbiA0MAp0dXJyZXQgMTIwIHdhaXQgMC41Cm1vdG9yIGV4dGVuZGVyIC0xNDAwIHBvc2l0aW9uCm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMjAKd2FpdCAxMDAKc2Vydm8gY2xhdyAwLjM1CnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41CndhaXQgMjAwCm1vdG9yIGV4dGVuZGVyIDAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiAxMDAKdHVycmV0IDAuOCBwb3dlcgp0dXJyZXQgMCB3YWl0IDAuMgp0dXJyZXQgMC40IHBvd2VyCm1vdG9yIGFuZ2xlIC0xNDAwIHBvc2l0aW9uCnNlcnZvIGludGFrZV9zZXJ2bzEgMQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAKc2Vydm8gY2xhdyAwLjQ1CmdvIHBvc2ggMzAgLTAuOCAwCnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41CndhaXQgMTAwCnNlcnZvIGNsYXcgMC42Mgp3YWl0IDMwMAptb3RvciBhbmdsZSAtNDUwIHBvc2l0aW9uCnR1cnJldCAxMjIKZ28gcG9zaCAwIC0wLjUgMAptb3RvciBhbmdsZSB3YWl0IHBvc2l0aW9uIDQwCnR1cnJldCAxMjIgd2FpdCAwLjUKbW90b3IgZXh0ZW5kZXIgLTE0MDAgcG9zaXRpb24KbW90b3IgZXh0ZW5kZXIgd2FpdCBwb3NpdGlvbiAyMAp3YWl0IDEwMApzZXJ2byBjbGF3IDAuMzUKc2Vydm8gaW50YWtlX3NlcnZvMSAwLjUKc2Vydm8gaW50YWtlX3NlcnZvMiAwLjUKd2FpdCAyMDAKbW90b3IgZXh0ZW5kZXIgMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDEwMAp0dXJyZXQgMC44IHBvd2VyCnR1cnJldCAwIHdhaXQgMC4yCnR1cnJldCAwLjQgcG93ZXIKbW90b3IgYW5nbGUgLTE0MDAgcG9zaXRpb24Kc2Vydm8gaW50YWtlX3NlcnZvMSAxCnNlcnZvIGludGFrZV9zZXJ2bzIgMApzZXJ2byBjbGF3IDAuNDUKZ28gcG9zaCAyMiAtMSAwCmdvIHBvc2ggMzQgMy42IDE5CnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41CndhaXQgMTAwCnNlcnZvIGNsYXcgMC42Mgp3YWl0IDMwMAptb3RvciBhbmdsZSAtNDUwIHBvc2l0aW9uCnR1cnJldCAxMjIKZ28gcG9zaCAyOCAtMSAwCmdvIHBvc2ggMCAtMSAwCm1vdG9yIGFuZ2xlIHdhaXQgcG9zaXRpb24gNDAKdHVycmV0IDEyMiB3YWl0IDAuNQptb3RvciBleHRlbmRlciAtMTQwMCBwb3NpdGlvbgptb3RvciBleHRlbmRlciB3YWl0IHBvc2l0aW9uIDIwCndhaXQgMTAwCnNlcnZvIGNsYXcgMC4zNQpzZXJ2byBpbnRha2Vfc2Vydm8xIDAuNQpzZXJ2byBpbnRha2Vfc2Vydm8yIDAuNQp3YWl0IDIwMAptb3RvciBleHRlbmRlciAwIHBvc2l0aW9uCnNlcnZvIGludGFrZV9zZXJ2bzEgMC41CnNlcnZvIGludGFrZV9zZXJ2bzIgMC41Cm1vdG9yIGV4dGVuZGVyIHdhaXQgcG9zaXRpb24gMTAwCnR1cnJldCAwCmdvIHBvc2ggMzAgLTAuOCAwCm1vdG9yIGFuZ2xlIC0xNDAwIHBvc2l0aW9uCm1vdG9yIGFuZ2xlIHdhaXQgcG9zaXRpb24=";

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class,"claw");
        intake1 = hardwareMap.get(Servo.class,"intake_servo1");
        intake2 = hardwareMap.get(Servo.class,"intake_servo2");
        extender = hardwareMap.get(DcMotor.class,"extender");
        angle = hardwareMap.get(DcMotor.class,"angle");
        intake1.setPosition(0.5);
        intake2.setPosition(0.5);

        turret = new Turret(hardwareMap,1);
        drive = new SampleMecanumDrive(hardwareMap);
        HardwareTesterInterpreter.initHWI(this,hardwareMap,telemetry,drive);

        cameraRecognition =  new CameraRecognition(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);
        cameraRecognition.tresh = 140;

        waitForStart();
        while (opModeIsActive()) {
            try {
                main();
            }
            catch (Exception e){
                telemetry.addLine(e.toString());
                telemetry.update();
            }
            break;
        }
    }

    private void Wait(int ms){
        try {
            Thread.sleep(ms);
        } catch (Exception ex){

        }
    }

    DcMotor extender,angle;
    Servo claw,intake1,intake2;

    void waitForAnglePosition(int tresh){
        while (true){
            int p = angle.getCurrentPosition();
            int t = angle.getTargetPosition();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
    }

    void waitForExtenderPosition(int tresh){
        while (true){
            int p = extender.getCurrentPosition();
            int t = extender.getTargetPosition();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
    }

    void waitForTurretAngle(double tresh){
        while (true){
            double p = turret.getAngle();
            double t = turret.getTargetAngle();
            if(p>= t-tresh && p<=t+tresh){
                break;
            }
            Wait(0);
        }
    }

    public class AutoTraj{
        public String type = "";
        public double x=0;
        public double y =0;
        public double heading = 0;
        public AutoTraj(String t,double _x,double _y,double _h){
            type = t;
            x = _x;
            y = _y;
            heading = _h;
        }

        public Trajectory build(){
            Trajectory trajectory = null;
            if(type == "lineToLinearHeading") {
                trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(x,y,heading))
                        .build();
            } else if(type == "splineTo"){
                trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(x,y),heading)
                        .build();
            }
            else if(type == "lineTo"){
                trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(x,y))
                        .build();
            }
            return trajectory;
        }
    }

    public static int target_freight = 3;
    public static double[] extraPosX = {27,29,31};
    AutoTraj[] freight_trajectories = {
            //new AutoTraj("")
    };

    private void main(){

        int freightCase = cameraRecognition.getCase();
        telemetry.addLine("Case: " + Integer.toString(freightCase));
        telemetry.update();

        Teleop2022_v2.fromAuto = 1;

        Thread t = new Thread(){
            @Override
            public void run(){
                while (opModeIsActive() && !isStopRequested()){
                    Teleop2022_v2.startPose = drive.getPoseEstimate();
                }
            }
        };


        if(freightCase == 1){
            HardwareTesterInterpreter.interpretScript(scriptLow,"base64");
        } else if(freightCase == 2){
            HardwareTesterInterpreter.interpretScript(scriptMid,"base64");
        } else if(freightCase == 3){
            HardwareTesterInterpreter.interpretScript(scriptHigh,"base64");
        }


        Teleop2022_v2.startPose = drive.getPoseEstimate();
    }
}
