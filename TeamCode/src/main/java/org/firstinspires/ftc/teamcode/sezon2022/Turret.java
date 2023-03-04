package org.firstinspires.ftc.teamcode.sezon2022;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Turret {

    public static double turretPower = 0.4;
    public static  int turretTicksCoef =  583 * 4;
    public DcMotorEx turretMotor;
    public DcMotorEx turretHelper;

    private void Sleep(int ms){
        try{
            Thread.sleep(ms);
        } catch (Exception e){

        }
    }

    public Turret(HardwareMap hw,int reset){
        turretMotor = hw.get(DcMotorEx.class,"turret");
        turretHelper = hw.get(DcMotorEx.class,"duck");

        if(reset == 1) {
            turretHelper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(turretPower);
        //turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double p){
        turretPower = p;
        turretMotor.setPower(p);
    }

    public void setPid(double p,double i,double d){
        PIDFCoefficients c = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        if(p == -1){
            p = c.p;
        } else if(i == -1){
            i = c.i;
        } else if(d == -1){
            d = c.d;
        }
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p,i,d,c.f));
    }

    public void setRotationAsync(double angle){

        int t1 = (int)((angle / 360.0) * (turretTicksCoef ));
        int t2 = (int)(-((360.0-angle) / 360.0 ) * (turretTicksCoef));
        int minDr1 = 10000;
        int minDr2 = 10000;
        int ot1 = 0;
        int ot2 = 0;
        int cT = turretMotor.getCurrentPosition();
        for(int i=0;i<10;i++){
            int a= Math.abs(cT - (t1 + i*turretTicksCoef));
            if(a < minDr1){
                ot1 = t1 + i * turretTicksCoef;
                minDr1 = a;
            }
        }
        for(int i=0;i<10;i++){
            int a= Math.abs(cT - (t2 - i*turretTicksCoef));
            if (a < minDr2){
                ot2 = t2 + i * turretTicksCoef;
                minDr2 = a;
            }
        }
        if (minDr1 < minDr2)
            turretMotor.setTargetPosition(ot1);
        else
            turretMotor.setTargetPosition(ot2);
    }

    /*
    public void setRotationAsync(double angle){

        int targetPos = (int) ((angle / 360.0) * (turretTicksCoef * 4));
        turretMotor.setTargetPosition(targetPos);
    }
     */

    public double getAngle(){
        return ((double)turretMotor.getCurrentPosition() / (turretTicksCoef)) * 360.0;
    }

    public double getAngleHelper(){
        return ((double)turretHelper.getCurrentPosition() / (8192)) * 360.0;
    }

    public double getTargetAngle(){
        return ((double)turretMotor.getTargetPosition() / (turretTicksCoef )) * 360.0;
    }

    public void setRotation(double angle){
        setRotationAsync(angle);
        while(turretMotor.getCurrentPosition() != turretMotor.getTargetPosition()){
            Sleep(1);
        }
    }
}
