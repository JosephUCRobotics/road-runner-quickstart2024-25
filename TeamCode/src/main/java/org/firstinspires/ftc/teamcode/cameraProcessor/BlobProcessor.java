package org.firstinspires.ftc.teamcode.cameraProcessor;

import org.firstinspires.ftc.teamcode.CameraVariables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class BlobProcessor {
    Mat mask;
    Mat displayed;
    public Mat processFrame(Mat frame, int id) {
        Mat blur = blur(frame);
        mask = applyColorFilter(blur);
        mask = applyErosion(mask);
        mask = applyDilation(mask);
        displayed = addBlackToMat(mask, blur);
        updateBlobPos(id);
        return displayed;
    }
    private Mat blur(Mat inputMat) {
        Mat outputMat = new Mat();
        Size size = new Size(CameraVariables.blurLevel, CameraVariables.blurLevel);
        Imgproc.blur(inputMat, outputMat, size);
        return outputMat;
    }
    private Mat applyColorFilter(Mat inputMat) {
        Mat ycrcbFrame = new Mat();
        Mat filteredFrame = new Mat();

        // Convert the input frame from the RGB color space to the YCrCb color space
        Imgproc.cvtColor(inputMat, ycrcbFrame, Imgproc.COLOR_RGB2YCrCb);

        // Define the lower and upper boundaries for the color range in the YCrCb color
        // space
        Scalar lowerBound = new Scalar(CameraVariables.lowerScalar1, CameraVariables.lowerScalar2, CameraVariables.lowerScalar3); // Lower boundary for the YCrCb color range
        Scalar upperBound = new Scalar(CameraVariables.upperScalar1, CameraVariables.upperScalar2, CameraVariables.upperScalar3); // Upper boundary for the YCrCb color range

        // Apply the color range filter in the YCrCb color space
        Core.inRange(ycrcbFrame, lowerBound, upperBound, filteredFrame);


        return filteredFrame;
    }
    private Mat applyErosion(Mat inputMat) {
        Mat outputMat = new Mat();
        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(CameraVariables.erodeLevel, CameraVariables.erodeLevel));
        Imgproc.erode(inputMat, outputMat, element);
        return outputMat;
    }
    public Mat applyDilation(Mat inputMat) {
        Mat outputMat = new Mat();
        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(CameraVariables.dilateLevel, CameraVariables.dilateLevel));
        Imgproc.dilate(inputMat, outputMat, element);
        return outputMat;
    }
    public Mat addBlackToMat(Mat blackMat, Mat inputMat) {
        Mat outputMat = new Mat();
        inputMat.copyTo(outputMat, blackMat); // Copy the original frame using
        // the filter as a mask}
        return outputMat;
    }
    void updateBlobPos(int id){
        // Find contours in the filtered frame
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the contour with the largest area
        double maxArea = 0;
        MatOfPoint maxContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
            }
        }

        // Draw a red rectangle around the largest contour and a red dot in the center
        if (maxContour != null) {
            org.opencv.core.Rect boundingRect = Imgproc.boundingRect(maxContour);
            if (CameraVariables.drawBox){
                Imgproc.rectangle(displayed, boundingRect.tl(), boundingRect.br(), new Scalar(0, 0, 255), 3);
            }

            int centerX = boundingRect.x + boundingRect.width / 2;
            int centerY = boundingRect.y;// + boundingRect.height / 2;
            if (CameraVariables.drawBox) {
                Imgproc.circle(displayed, new org.opencv.core.Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);
            }

            // Calculate the x and y distances from the green dot (center of the screen) to
            // the red dot
            int screenCenterX = CameraVariables.cameraResolution.getHeight() / 2;
            int screenCenterY = CameraVariables.cameraResolution.getWidth() / 2;
            if (CameraVariables.drawBox) {
                Imgproc.circle(displayed, new org.opencv.core.Point(screenCenterX, screenCenterY), 5, new Scalar(0, 255, 0), -1);
            }
            int distanceX = centerX - screenCenterX;
            int distanceY = centerY - screenCenterY;
            String distanceInfo = "Distance: X=" + distanceX + ", Y=" + distanceY;
            if (CameraVariables.drawBox){
            Imgproc.putText(displayed, distanceInfo, new org.opencv.core.Point(20, 75), Imgproc.FONT_HERSHEY_SIMPLEX,
                    1, new Scalar(0, 255, 0), 3);
            }

            if (id == 1) {
                CameraVariables.distanceX1 = distanceX;
                CameraVariables.distanceY1 = distanceY;
            } else {
                CameraVariables.distanceX2 = distanceX;
                CameraVariables.distanceY2 = distanceY;
            }

            // Add text information about the width and height of the rectangle below the x
            // and y dimensions
            String rectInfo = "Width: " + boundingRect.width + ", Height: " + boundingRect.height;
            if (CameraVariables.drawBox) {
                Imgproc.putText(displayed, rectInfo, new org.opencv.core.Point(20, 150), Imgproc.FONT_HERSHEY_SIMPLEX, 1,
                        new Scalar(0, 0, 255), 3);
            }
        }
    }
    double[] getDistanceToBlobWithCam1(){
        double angleRight = -CameraVariables.distanceX1 / 14;
        double angleUp = CameraVariables.distanceY1 / 14-CameraVariables.camOffset;
        double c = CameraVariables.camHeight;
        double x = Math.tan(Math.toRadians(90 + angleUp)) * c;
        double y = Math.tan(Math.toRadians(angleRight)) * x;
        double[] out = {2, 4};
        return out;
    }
}
