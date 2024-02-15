package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class NewVision implements VisionProcessor{
  //g  private DrawRectangleProcessor drawRectangleProcessor;
    Telemetry telemetry;

    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE
    }
    public NewVision(Telemetry t){
        telemetry = t;
    }
    private volatile NewVision.Location location = NewVision.Location.RIGHT;
    Mat mat = new Mat();
    static double PERCENT_COLOR_THRESHOLD = 0.18;
    static final Rect MIDDLE_ROI = new Rect(
            new Point(30, 135),
            new Point(90, 175));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 135),
            new Point(200, 175));
    private VisionPortal visionPortal;

    public void init(int width, int height, CameraCalibration calibration) {

    }


    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(173, 150, 75);
        Scalar highHSV = new Scalar(179, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        right.release();
        middle.release();


        boolean tseRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean tseMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        if (tseRight) {
             location = NewVision.Location.RIGHT;
            telemetry.addData("TSE Location: ", "RIGHT");
        }
        else if(tseMiddle) {
            location = NewVision.Location.MIDDLE;
            telemetry.addData("TSE Location: ", "MIDDLE");
        }
        else {
            location = NewVision.Location.LEFT;
            telemetry.addData("TSE not detected; Location: ", "LEFT");
        }

        telemetry.update();


        // change the img back to RGB
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        // below will make the boxes Red if no TSE exists, and Green if it does within the rectangle box
        Scalar noTSE = new Scalar(255, 0, 0);
        Scalar tseDetected = new Scalar(0, 255, 0);

        // depending on where the TSEe is, or where it isn't, the color of the rectangle will change
        Imgproc.rectangle(mat, RIGHT_ROI, location == NewVision.Location.RIGHT? tseDetected:noTSE);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == NewVision.Location.MIDDLE? tseDetected:noTSE);

        return mat;
    }
    public Location getLocation() {

        return location;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Scalar noTSE = new Scalar(255, 0, 0);
        Scalar tseDetected = new Scalar(0, 255, 0);

        // depending on where the TSEe is, or where it isn't, the color of the rectangle will change
        Imgproc.rectangle(mat, RIGHT_ROI, location == NewVision.Location.RIGHT? tseDetected:noTSE);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == NewVision.Location.MIDDLE? tseDetected:noTSE);
    }


}
