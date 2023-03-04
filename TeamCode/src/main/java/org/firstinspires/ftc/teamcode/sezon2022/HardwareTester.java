package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashMap;


/*

go posh 25 0 -20
go rot -40
go posh 62 5 -90

 */

@Config
@Autonomous
public class HardwareTester extends LinearOpMode {

    public static int useRR = 1;
    FtcDashboard dashboard;

    HashMap<String,DcMotor> motorHashMap;

    double getBatteryVoltage(){
        double result = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor: hardwareMap.voltageSensor){
            double voltage = sensor.getVoltage();
            if(voltage>0){
                result = Math.min(result,voltage);
            }
        }
        return result;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        //turret = new Turret(hardwareMap,1);
        if(useRR == 1) {
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        waitForStart ();
        while (opModeIsActive ()) {
            main();
            break;
        }
    }

    void Sleep(int ms){
        try{
            Thread.sleep(ms);
        }catch (InterruptedException e){

        }
    }


    public String request(){
        try {
            URL url = new URL("http://192.168.43.219:4444/cmd");
            HttpURLConnection con = (HttpURLConnection) url.openConnection();
            con.setRequestMethod("GET");
            BufferedReader in = new BufferedReader(new InputStreamReader(con.getInputStream()));
            String line;
            String s = "";
            while ((line = in.readLine()) != null) {
                s += line + "\n";
            }
            con.disconnect();
            return s;
        }
        catch (Exception e){
            telemetry.addData("error",e.toString());
            telemetry.update();
            return "";
        }
    }

    Servo intakeHelper1,intakeHelper2;
    SampleMecanumDrive drive;
    float drivePower = 0.3f;
    boolean traj = false;

    String currentScript="";

    public void interpretScript(String script){
        currentScript = script;
        String[] lines = script.split("\\n");
        String a="";
        for(int i=0;i<lines.length;i++) {
            if(lines[i].equals(""))
                continue;
            a += lines[i] + "@";
        }
        telemetry.addData("data: ",a);
        telemetry.update();
        for(int i=0;i<lines.length;i++){
            if(lines[i].equals(""))
                continue;
            if(opModeIsActive() == false || isStopRequested())
                break;
            interpretCommand(lines[i]);
        }
    }

    public void interpretCommand(String s){
        String[] args = s.split(" ");
        if(args[0].equals("go")){
            traj =true;
            if(args[1].equals("fw") || args[1].equals("forward")){
                double distance = Double.valueOf(args[2]);
                Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                        .forward(distance/2.54d)
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("bw") || args[1].equals("backward")){
                double distance = Double.valueOf(args[2]);
                Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                        .back(distance/2.54d)
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("left") || args[1].equals("l")){
                double distance = Double.valueOf(args[2]);
                Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(distance/2.54d)
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("right") || args[1].equals("r")){
                double distance = Double.valueOf(args[2]);
                Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(distance/2.54d)
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("rot")){
                double angle = Double.valueOf(args[2]);
                drive.turn(Math.toRadians(angle));
            } else if(args[1].equals("test")) {
                double distance = Double.valueOf(args[2]);
                //DriveConstants.setStraight();
                Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(distance / 2.54d)
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("pos")){
                Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(Double.valueOf(args[2]),Double.valueOf(args[3])))
                        .build();
                drive.followTrajectory(trajectory);
            }
            else if(args[1].equals("posh")){
                Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(Double.valueOf(args[2]),Double.valueOf(args[3]),Math.toRadians(Double.valueOf(args[4]))))
                        .build();
                drive.followTrajectory(trajectory);
            }
            else if(args[1].equals("seq")){
                TrajectoryBuilder trajectory_builder = drive.trajectoryBuilder(drive.getPoseEstimate());
                //.lineToLinearHeading(new Pose2d(Double.valueOf(args[2]),Double.valueOf(args[3]),Math.toRadians(Double.valueOf(args[4]))));
                String f = "";
                for(int i=2;i<args.length;i++){
                    String[] args3= args[i].split("/");
                    String[] args2 = args3[1].split(",");
                    String type = args3[0];
                    if(type.equals("posh")) {
                        trajectory_builder.lineToLinearHeading(new Pose2d(Double.valueOf(args2[0]), Double.valueOf(args2[1]), Math.toRadians(Double.valueOf(args2[2]))));
                    } else if(type.equals("pos-const")){
                        trajectory_builder.lineToConstantHeading(new Vector2d(Double.valueOf(args2[0]), Double.valueOf(args2[1])));
                    }
                    f += "traj " + type +";";
                }
                telemetry.addData("aaa",f);
                telemetry.update();
                Trajectory trajectory = trajectory_builder.build();
                drive.followTrajectory(trajectory);
             }
            else if(args[1].equals("spline")) {
                Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(Double.valueOf(args[2]), Double.valueOf(args[3])), Math.toRadians(Double.valueOf(args[4])))
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("spline_heading")){
                Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(Double.valueOf(args[2]),Double.valueOf(args[3]),Math.toRadians(Double.valueOf(args[4]))),Math.toRadians(Double.valueOf(args[5])))
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("spline_spline")){
                Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(Double.valueOf(args[2]),Double.valueOf(args[3]),Math.toRadians(Double.valueOf(args[4]))),Math.toRadians(Double.valueOf(args[5])))
                        .build();
                drive.followTrajectory(trajectory);
            }
            else if(args[1].equals("set")){
                if(args[2].equals("speed")){
                    DriveConstants.MAX_ACCEL = Double.parseDouble(args[3]);
                    DriveConstants.MAX_VEL = Double.parseDouble(args[3]);
                } else if(args[2].equals("pid")){

                }
            }
            drive.update();
            traj=false;
        }
        else if(args[0].equals("servo")){
            try {
                ServoImplEx sv = hardwareMap.get(ServoImplEx.class,args[1]);
                if(args[2].equals("close")){
                    sv.getController().pwmDisable();
                } else if(args.length == 5){
                    if(args[3].equals("after")){
                        Thread th = new Thread() {
                            @Override
                            public void run() {
                                final int x = Integer.parseInt(args[4]);
                                Sleep(x);
                                sv.setPosition(Float.parseFloat(args[2]));
                            }
                        };
                        th.start();
                    }
                }
                else {
                    //sv.getController().pwmEnable();
                    sv.setPosition(Float.parseFloat(args[2]));
                }
            }catch (Exception a){
                telemetry.addLine(a.toString());
                telemetry.update();
            }
        }
        else if(args[0].equals("motor")){
            // motor mosor 100 position
                try {
                    final DcMotor dc = hardwareMap.get(DcMotor.class, args[1]);
                    if(args[3].equals("position")) {
                        if(args[2].equals("wait")) {
                            int tresh = 0;
                            if(args.length>4){
                                tresh = Integer.parseInt(args[4]);
                            }
                            int p = dc.getCurrentPosition();
                            int t = dc.getTargetPosition();
                            while (!(p >= t - tresh && p <= t + tresh)){
                                p = dc.getCurrentPosition();
                                telemetry.addData("From " + args[1],p);
                                telemetry.update();
                                //telemetry.addData(args[1],p);
                                //telemetry.update();
                                Sleep(5);
                            }
                            telemetry.addLine("Wait finish with " + Integer.toString(p));
                            telemetry.update();
                        } else if(args.length == 6){
                            if(args[4].equals("after")){
                                Thread th = new Thread() {
                                    @Override
                                    public void run() {
                                        final int x = Integer.parseInt(args[5]);
                                        Sleep(x);
                                        dc.setTargetPosition(Integer.parseInt(args[2]));
                                        dc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                    }
                                };
                                th.start();
                            }
                        }
                        else {
                            dc.setTargetPosition(Integer.parseInt(args[2]));
                            dc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                    } else if(args[3].equals("power")){
                        dc.setPower(Float.parseFloat(args[2]));
                    } else if(args[3].equals("normal")){
                        dc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    else if(args[3].equals("reset_position")){
                        dc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    } else if(args[3].equals("fw")){
                        dc.setDirection(DcMotor.Direction.FORWARD);
                    } else if(args[3].equals("bw")){
                        dc.setDirection(DcMotor.Direction.REVERSE);
                    }
                    //telemetry.addData("motor " + args[1] +" update: ",args[3] +"/"+args[2]);
                    //telemetry.update();
                } catch (Exception a) {
                    telemetry.addLine(a.toString());
                    telemetry.update();
                    Sleep(5000);
                }
        }
        else if(args[0].equals("rr")){
            try {
                if (args[1].equals("speed")) {
                    DriveConstants.MAX_VEL = Double.parseDouble(args[2]);
                } else if (args[1].equals("accel")) {
                    DriveConstants.MAX_ACCEL = Double.parseDouble(args[2]);
                } else if (args[1].equals("kv")) {
                    DriveConstants.kV = Double.parseDouble(args[2]);
                } else if (args[1].equals("ka")) {
                    DriveConstants.kA = Double.parseDouble(args[2]);
                }
            }catch (Exception a){
                telemetry.addLine(a.toString());
                telemetry.update();
            }
        }
        else if(args[0].equals("wait")){
            try{
                Sleep(Integer.parseInt(args[1]));
            } catch (Exception a){
                telemetry.addLine(a.toString());
                telemetry.update();
            }
        } else if(args[0].equals("print")){
            /*
                String oName = args[2];
                if(args[1].equals("motor_pos")){

                } else if(args[1].equals("servo_pos")) {

                }
                if(args.length >=4){
                    String[] a = args[3].split("x");
                    final int loop = Integer.parseInt(a[0]);
                    final int time = Integer.parseInt(a[1]);
                    Thread th = new Thread(){
                        int _loop=loop;
                        int _time=time;
                        @Override
                        public void run(){
                            for(int i=0;i<_loop;i++){
                                telemetry.addData()
                                Sleep(_time);
                            }
                        }
                    };
                }

             */
        }
    }

    public void main(){

        /*
        intakeHelper1 = hardwareMap.get(Servo.class,"intakeHelper1");
        intakeHelper2 = hardwareMap.get(Servo.class,"intakeHelper2");
        */

        Thread th = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive() && !isStopRequested()){
                    String s = request();
                    if(s == "" || s == null) {
                        Sleep(100);
                        continue;
                    }

                    try {
                        interpretScript(s);
                    }catch (Exception a){
                        telemetry.addLine(a.toString());
                        telemetry.update();
                    }
                    Sleep(100);
                }
                telemetry.addLine("Hardware tester close");
                telemetry.update();
            }
        };
        th.start();




        while (opModeIsActive()) {
           // TelemetryPacket packet = new TelemetryPacket();
           // packet.put("Voltage",getBatteryVoltage());
           // dashboard.sendTelemetryPacket(packet);
            //telemetry.addData("Lines:",currentScript.split("\\n").length);
            //telemetry.update();
            if(useRR == 1) {

                /*
                Pose2d poseEstimate = drive.getPoseEstimate();

                telemetry.addData("Voltage",getBatteryVoltage());
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();

                 */

            }
            if(!traj && useRR == 1) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * drivePower,
                                -gamepad1.left_stick_x * drivePower,
                                -gamepad1.right_stick_x * drivePower
                        )
                );
                drive.update();
            }
        }

        th.interrupt();


    }
}