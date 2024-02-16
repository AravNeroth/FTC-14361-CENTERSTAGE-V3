package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Detection;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class NewVision implements VisionProcessor {
    //g  private DrawRectangleProcessor drawRectangleProcessor;
    Telemetry telemetry;
    // private Rect rectLeft = new Rect(40, 230, 80, 100);
    private Rect rectMiddle = new Rect(349, 325, 90, 110);

    private Rect rectRight = new Rect(366, 25, 100, 80);


    StartingPosition selection = StartingPosition.LEFT;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public NewVision(Telemetry telem) {
        telemetry = telem;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        double colRectMiddle = getAvgColor(hsvMat, rectMiddle);
        double colRectRight = getAvgColor(hsvMat, rectRight);

        telemetry.addData("right sat: ", satRectRight);
        telemetry.addData("mid sat: ", satRectMiddle);

        telemetry.addData("right col: ", colRectRight);
        telemetry.addData("middle col: ", colRectMiddle);



        if ((colRectMiddle > colRectRight) && (colRectMiddle - colRectRight > 35)) {
            selection = StartingPosition.CENTER;

        } else if ((colRectRight > colRectMiddle) && (colRectRight - colRectMiddle > 35)) {
            selection = StartingPosition.RIGHT;

        } else {
            selection = StartingPosition.LEFT;
        }
        telemetry.addData("Selection decided: ", selection );
        telemetry.update();

        return selection;
    }


    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    protected double getAvgColor(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[0];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);

        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);

        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelected = new Paint();
        nonSelected.setStrokeWidth(scaleCanvasDensity * 4);
        nonSelected.setStyle(Paint.Style.STROKE);
        nonSelected.setColor(Color.RED);

        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (StartingPosition) userContext;

        switch (selection) {

            case RIGHT:
                canvas.drawRect(drawRectangleMiddle, nonSelected);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case CENTER:
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelected);
                break;
            case LEFT:
                canvas.drawRect(drawRectangleMiddle, nonSelected);
                canvas.drawRect(drawRectangleRight, nonSelected);
                break;

        }

    }

    public StartingPosition getStartingPosition() {
        return selection;
    }

    public enum StartingPosition {

        LEFT,
        RIGHT,
        CENTER
    }
}