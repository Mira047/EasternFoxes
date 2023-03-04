package org.firstinspires.ftc.teamcode.sezon2023;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Urgenta Dreapta", group="Linear Opmode")
@Config
public class UrgentaDreapta extends LinearOpMode {


    SampleMecanumDrive drive;



    DistanceSensor distanceSensor;
    DistanceSensor corectieLaterala;
    DistanceSensor corectieFata;
BNO055IMU imu;


    public double getRawExternalHeading() {
        return Math.toDegrees(imu.getAngularOrientation().firstAngle);
    }

    public static double directieRotireStanga = 1;
    public static double directieRotireDreapta = -1;
    public static double directieMersStanga = 1;
    public  static  double directieMersDreapta = -1;

    public static double vitezaRotire = 0.25;

    public static double vitezaMers = 0.45;


    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);


        //HardwareTesterInterpreter.initHWI(this,hardwareMap,telemetry,drive);

        //corectieFata = hardwareMap.get(DistanceSensor.class,"corectieFata");
        corectieLaterala = hardwareMap.get(DistanceSensor.class,"corectieLaterala");

        double ang=0;
        while (!opModeIsActive() && !isStopRequested()){
            ang = getRawExternalHeading();
            telemetry.addData("ang",ang);
            telemetry.update();
        }

        waitForStart ();
        drive = new SampleMecanumDrive(hardwareMap);

         ang = getRawExternalHeading();
         double directieRotire = 0;
         if(ang<0){
             directieRotire = directieRotireStanga;
         }
         else{
             directieRotire = directieRotireDreapta;
         }
        while (ang >= 0.25 || ang <= -0.25){
            drive.setWeightedDrivePower(new Pose2d(
                    0,
                    0,
                    vitezaRotire * directieRotire
            ));
            drive.update();
            ang = getRawExternalHeading();
        }
        telemetry.addData("ang",ang);
        telemetry.update();

        double dist=0;
        dist = corectieLaterala.getDistance(DistanceUnit.CM);

        double directieMers = 0;
        if(dist < 100)
        {
            directieMers = directieMersStanga;
        } else {
            directieMers = directieMersDreapta;
        }

        drive.update();
        if(directieMers == directieMersStanga){
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .strafeLeft((100 - dist)/2.54)
                    .build();
            drive.followTrajectory(trajectory);
        } else {
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .strafeRight((dist - 100)/2.54)
                    .build();
            drive.followTrajectory(trajectory);
        }
        /*

        while (dist< 97 || dist > 103){
            drive.setWeightedDrivePower(new Pose2d(
                    0,
                    directieMers * vitezaMers,
                    0
            ));
            drive.update();
            dist = corectieLaterala.getDistance(DistanceUnit.CM);
        }

        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        drive.update();

         */

    }



    void Wait(int ms){
        try{
            Thread.sleep(ms);
        }catch (Exception ex){

        }
    }


    private  void main(){


    }
}
