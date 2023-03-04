package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Controller {

    public double power ( double x, double y , double spd) {
        double l = Math.sqrt ( x * x + y * y );
        if (Math.abs (l) > 0.05 )
            return Math.abs(l*spd);
        return 0;
    }

    public double New_angle(double y, double x) {
        double Angle = Math.atan2 (y, x);
        if(Angle < 0)
            Angle += 6.2831855;
        return Math.toDegrees ((Angle + 1.5707964) % 6.2831855);
    }
}
