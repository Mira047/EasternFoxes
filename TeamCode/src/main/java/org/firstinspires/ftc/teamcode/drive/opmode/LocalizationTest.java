package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

   // Turret turret;
    public static int tankTurret = 1;
    Pose2d lastPose = null;
    //DcMotor angle,extender;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    AngularVelocity omegas;

    Servo ghidaj;

    @Override
    public void runOpMode() throws InterruptedException {

        ghidaj = hardwareMap.get(Servo.class,"ghidaj");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // get calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// retrieve gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

         */


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turret.setPower(0.8);

        waitForStart();


        while (!isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("heading deg",Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("ghidaj pos",ghidaj.getPosition());
            /*
            telemetry.addData("imu external heading",Math.toDegrees(drive.getRawExternalHeading()));
            Acceleration vel = drive.imu.getLinearAcceleration();
            Position p= drive.imu.getPosition();

            telemetry.addData("imu p1", p.x);
            telemetry.addData("imu p2",p.y);



            telemetry.addData("imu vel X", vel.xAccel);
            telemetry.addData("imu vel Y",vel.yAccel);
            telemetry.addData("imu vel Z", vel.zAccel);

             */



            //telemetry.addData("turret",turret.getAngle());
            /*
            if(lastPose != null && tankTurret == 1){
                double robotX = poseEstimate.getX();
                double robotY = poseEstimate.getY();

                double wX = -22.4;
                double wY = 40.1;


                double x = wX - robotX;
                double y = wY - robotY;

                double rot = Math.toDegrees(poseEstimate.getHeading());
                if(rot>180){
                    rot = -(360-rot);
                }
                turret.setRotationAsync(atan2(y,x) * 180 / PI + -rot);



                turret.setRotationAsync(-Math.toDegrees(poseEstimate.getHeading()));
                //double dif = Math.toDegrees(poseEstimate.getHeading()) - Math.toDegrees(lastPose.getHeading());
            }

             */
            telemetry.update();
            lastPose = poseEstimate;
        }
    }
}
