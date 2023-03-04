package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class HardwareTesterInterpreter {

    public static int useRR = 1;
    static Telemetry telemetry;
    static LinearOpMode activeOpMode;
    static HardwareMap hardwareMap;
    public static boolean traj = false;

    static void Sleep(int ms){
        try{
            Thread.sleep(ms);
        }catch (InterruptedException e){

        }
    }

    static SampleMecanumDrive drive;

    public static void interpretScript(String script, String mode){
        if(script == ""){
            telemetry.addData("error","null script");
            telemetry.update();
            return;
        }
        if(mode == "base64"){
            script = new String(android.util.Base64.decode(script,android.util.Base64.DEFAULT));
        }
        String[] lines = script.split("\\n");
        String a="";
        for(int i=0;i<lines.length;i++) {
            a += lines[i] + "@";
        }
        telemetry.addData("data: ",a);
        telemetry.update();
        for(int i=0;i<lines.length;i++){
            if(lines[i] == "")
                continue;
            if(activeOpMode.opModeIsActive() == false || activeOpMode.isStopRequested())
                break;
            interpretCommand(lines[i]);
        }
    }

    public static void initHWI(LinearOpMode opMode,HardwareMap hw,Telemetry tl,SampleMecanumDrive d){
        hardwareMap = hw;
        telemetry = tl;
        activeOpMode = opMode;
        //turret = tr;
        drive = d;
    }

    public static void interpretCommand(String s){
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
            } else if(args[1].equals("posh-seq")){
                TrajectoryBuilder trajectory_builder = drive.trajectoryBuilder(drive.getPoseEstimate());
                        //.lineToLinearHeading(new Pose2d(Double.valueOf(args[2]),Double.valueOf(args[3]),Math.toRadians(Double.valueOf(args[4]))));
                for(int i=2;i<args.length;i++){
                    String[] args2= args[i].split(",");
                    trajectory_builder.lineToLinearHeading(new Pose2d(Double.valueOf(args2[0]),Double.valueOf(args2[1]),Math.toRadians(Double.valueOf(args2[2]))));
                }
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
                        .splineToLinearHeading(new Pose2d(Double.valueOf(args[2]),Double.valueOf(args[3]),Double.valueOf(args[4])),Math.toRadians(Double.valueOf(args[5])))
                        .build();
                drive.followTrajectory(trajectory);
            } else if(args[1].equals("set")){
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
                } else {
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
                DcMotor dc = hardwareMap.get(DcMotor.class, args[1]);
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
                            telemetry.addData(args[1],p);
                            telemetry.update();
                            Sleep(1);
                        }
                    } else {
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
                telemetry.addData("motor " + args[1] +" update: ",args[3] +"/"+args[2]);
                telemetry.update();
            } catch (Exception a) {
                telemetry.addLine(a.toString());
                telemetry.update();
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

}