package org.firstinspires.ftc.teamcode.sezon2022;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipeLineDetector extends OpenCvPipeline {
    public enum Status{
        VERDE1,
        ROSU2,
        ALBASTRU3
    }
    public Status caz = Status.VERDE1;
    public Telemetry telemetry;
    public Rect r1;
    public int xA = 480, yA = 210;
    public int xB = 550, yB = 280;
    PipeLineDetector(int xAI,int yAI,int xBI,int yBI)
    {
        xA=xAI;
        yA=yAI;
        xB=xBI;
        yB=yBI;
    }

    Mat HSV = new Mat();
    Mat copVerde = new Mat();
    Mat copRosu = new Mat();
    Mat bunVerde = new Mat();
    Mat bunRosu = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        copVerde = input;
        copRosu = input;


            final Point A = new Point(xA, yA);
            final Point B = new Point(xB, yB);
            double arie = (B.x - A.x) * (B.y - A.y);

            Imgproc.cvtColor(copVerde, HSV, Imgproc.COLOR_RGB2HSV);
            Scalar lowGreen = new Scalar(30, 50, 20);
            Scalar highGreen = new Scalar(80, 255, 255);
            Core.inRange(HSV, lowGreen, highGreen, bunVerde);
            Mat verde = bunVerde.submat(new Rect(A, B));
            Scalar sumVerde = Core.sumElems(verde);
            Imgproc.cvtColor(copRosu, HSV, Imgproc.COLOR_RGB2HSV);
            Scalar lowRed = new Scalar(175, 100, 20);
            Scalar highRed = new Scalar(180, 255, 255);
            Core.inRange(HSV, lowRed, highRed, bunRosu);
            Mat rosu = bunRosu.submat(new Rect(A, B));
            Scalar sumRosu = Core.sumElems(rosu);

            if (sumVerde.val[0] / arie > 0.5) caz = Status.VERDE1;
            else if (sumRosu.val[0] / arie > 0.389) caz = Status.ROSU2;
            else caz = Status.ALBASTRU3;
            Imgproc.rectangle(input, A, B, new Scalar(0, 0, 255), 3);

        return input;
    }
}