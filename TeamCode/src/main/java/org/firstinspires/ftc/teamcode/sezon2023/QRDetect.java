package org.firstinspires.ftc.teamcode.sezon2023;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.sezon2022.PipeLineDetector;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

//import org.openftc.easyopencv.


public class QRDetect {
    public  OpenCvWebcam webcam;
    QRCodeDetector qr_detector = new QRCodeDetector();
    Point topLeft,topRight,topMid,bottomLeft,bottomMid,bottomRight;
    Point cropTop;
    Point cropBottom;
    Point midLeft;
    Point midRight;
    public PipeLineDetector detector;
    public  int rezX=640;
    public  int rezY=480;
    int mean1,mean2,mean3;
    int v1=0,v2=0,v3=0;
    public int Case=0;
    public String mode="";
    int action;
    public String qr_text = "";
    public String last_qr = "";
    public int tresh = 140;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public QRDetect(HardwareMap map, Telemetry tel){
        hardwareMap = map;
        telemetry = tel;
    }

    public String getCase(){
        return qr_text;
        /*
        if(qr_text == "") {
            return last_qr;
        } else{
            return qr_text;
        }*/
    }

    public void initCamera(){
        topLeft = new Point(0 ,0);
        topMid= new Point(rezX/3 + 1,rezY/6);
        topRight= new Point(rezX/3 * 2 + 1,rezY/6);
        bottomLeft = new Point(rezX/3,rezY - rezY/2);
        bottomMid= new Point(rezX/3 * 2,rezY - rezY/2);
        bottomRight= new Point(rezX,rezY);
        cropTop = new Point(0 + rezX/6,0 + rezY/6);
        cropBottom = new Point(rezX - rezX/6,rezY - rezY/6);
        //midLeft = new Point(0,0);
        //midRight = new Point(rezX-1,rezX-1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        //qr_detector.
        // webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDevice();


    }

    void sleep(int ms){
        try{
            Thread.sleep(ms);
        }catch (InterruptedException e){

        }
    }

    public void recognize(){
        sleep(1000);
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();
    }

    public  void stop(){
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();
    }

    public void start(){
        webcam.startStreaming(rezX, rezY, OpenCvCameraRotation.UPRIGHT);
    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        Mat m=new Mat();

        Scalar blue = new Scalar(0,0,255);
        Scalar red = new Scalar(255,0,0);

        @Override
        public Mat processFrame(Mat input)
        {

            //crop = input.submat(new Rect(cropTop, cropBottom));
            Mat crop=new Mat();
            //Imgproc.cvtColor(input,m,Imgproc.COLOR_BGR2GRAY);
            List<String> l = new ArrayList<>();
            qr_detector.detectAndDecodeMulti(input,l,crop);
            //String t = qr_detector.detectAndDecode(input,crop);

            if(crop.empty() == false) {
                for (int i = 0; i < crop.cols(); i++) {
                    Point pt1 = new Point(crop.get(0, i));
                    Point pt2 = new Point(crop.get(0, (i + 1) % 4));
                    Imgproc.line(input, pt1, pt2, red, 3);
                }
            }
            //if(input.cols()> cropTop.y && ) {
             //   Imgproc.rectangle(input, cropTop, cropBottom, blue, 3);
            //}
            //telemetry.addData("decoded ",t);
            if(l.size()>0) {
                String a = "";
                for(int i=0;i<l.size();i++){
                    a += l.get(i) + "|";
                }
                String t = a;
                qr_text = t;
                if (!t.equals("")) {
                    telemetry.addLine("+ " + t);
                    telemetry.update();
                        last_qr = t;
                }
            }
            //telemetry.update();
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}

