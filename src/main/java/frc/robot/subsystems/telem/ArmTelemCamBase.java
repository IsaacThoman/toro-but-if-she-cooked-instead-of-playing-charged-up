package frc.robot.subsystems.telem;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import static frc.robot.Constants.ArmConstants.*;

import java.util.ArrayList;
import java.util.List;

public class ArmTelemCamBase {

    Thread visionThread;
    static int targetFramerate = 6;

    double armAngStg1Rad = Units.degreesToRadians(45);
    double armAngStg2Rad = Units.degreesToRadians(-25);

    public ArmTelemCamBase(){
        if(RobotBase.isSimulation()){
            targetFramerate = 15;
        }

        visionThread = new Thread(
                ()-> {
                    CvSource outputStream = CameraServer.putVideo("ArmTelem",480,360);
                    Mat mat = new Mat(360,480,16);

                    while(!Thread.interrupted()){
                        this.drawThings(mat);
                        outputStream.putFrame(mat);

                        try {Thread.sleep((int)(1000.0/targetFramerate));} catch (InterruptedException e) {}
                    }

                });

        visionThread.setName("MB_ArmTelemCam");
        visionThread.setDaemon(true);
        visionThread.start();
    }

    private void drawThings(Mat mat){
        clearScreen(mat);
        drawArms(mat,armAngStg1Rad,armAngStg2Rad);
    }

    private void drawArms(Mat mat,double angStg1, double angStg2){ //angles in radians always
        double originX = stageOnePivotCoordinate[0];
        double originY = stageOnePivotCoordinate[1];

        double x1 = originX; double y1 = originY;

        double x2 = originX + stageOneLength*Math.cos(angStg1);
        double y2 = originY + stageOneLength*Math.sin(angStg1);

        double x3 = x2+ stageTwoLength*Math.cos(angStg1+angStg2);
        double y3 = y2+ stageTwoLength*Math.sin(angStg1+angStg2);

        Imgproc.line(mat, metersPosToPixelsPos(new Point(x1,y1)),metersPosToPixelsPos(new Point(x2,y2)),new Scalar(255,255,255),2);
        Imgproc.line(mat, metersPosToPixelsPos(new Point(x2,y2)),metersPosToPixelsPos(new Point(x3,y3)),new Scalar(255,255,255),2);


    }
    private void clearScreen(Mat mat){
        List<MatOfPoint> pointList1 = new ArrayList<MatOfPoint>();
        pointList1.add(new MatOfPoint(
                new Point(0,0),
                new Point(0,480),
                new Point(768,480),
                new Point(768,0)
        ));
        Imgproc.fillPoly(mat,pointList1,new Scalar(0,0,0));

    }

    //directly stolen :p
    public  Point metersPosToPixelsPos(Point posInMeters){
        double centerX = 480/2;
        double centerY = 360/2;
        double ang = Math.toRadians(180);
        posInMeters.x = -posInMeters.x; //what?
        //     posInMeters.x += SmartDashboard.getNumber("TCamX",0);
        //   posInMeters.y += SmartDashboard.getNumber("TCamY",0);

        posInMeters.x += 1.0; //1.25
        posInMeters.y += -0.9;

        //   double zoom = SmartDashboard.getNumber("TCamZoom",1) * 0.6  ;
        double zoom = 3;
        //   ang+= Math.toRadians(SmartDashboard.getNumber("TCamAngle",0));
        double outX = centerX;
        double outY = centerY;
        outX+= posInMeters.x * 60;
        outY+= posInMeters.y * 60;

        double dist = Math.sqrt(Math.pow(outX - centerX ,2) + Math.pow(outY -centerY,2)) * zoom;
        double dir = Math.atan2(outY -centerY, outX - centerX);
        outX = centerX + dist * Math.cos(dir+ang);
        outY = centerY + dist * Math.sin(dir+ang);

        return new Point (outX,outY);
    }

}
