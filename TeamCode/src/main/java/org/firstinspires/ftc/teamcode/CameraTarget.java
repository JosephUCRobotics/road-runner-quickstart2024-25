package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class CameraTarget {
    double targetsFieldX = 0;
    double targetsFieldY = 0;
    double targetsFieldZ = 0;
    int blurLevel = 10;
    int lowerScalar1 = 0;
    int lowerScalar2 = 150;
    int lowerScalar3 = 0;
    int upperScalar1 = 150;
    int upperScalar2 = 255;
    int upperScalar3 = 255;
    int erodeLevel = 15;
    int dilateLevel = 15;

    public void setPosition(double X, double Y){
        targetsFieldX = X;
        targetsFieldY = Y;
    } public void setPosition(double X, double Y, double Z){
        targetsFieldX = X;
        targetsFieldY = Y;
        targetsFieldZ = Z;
    }

    public double[] getDistanceFromCamera(){
        Mat cameraStream = CameraVariables.cameraStream1;
        Mat blur = blur(cameraStream);
        Mat mask = applyColorFilter(blur);
        mask = applyErosion(mask);
        mask = applyDilation(mask);
        org.opencv.core.Rect blobRect = getBlobsBoundingRect(mask);
        double[] XandYInPixels = getBlobsPixelPosition(blobRect);
        CameraVariables.matForDisplay = getMatForDisplay(mask, blur, blobRect);

        // the negative sings are needed because the starting at the center positive y is down and positive x is to the right
        double angleRight = -XandYInPixels[0] / 14;
        double angleUp = -XandYInPixels[1] / 14-CameraVariables.camOffset;
        double c = CameraVariables.camHeight;
//        double x = Math.tan(Math.toRadians(90 + angleUp)) * c;
        double x = -((Math.log10(XandYInPixels[1] / 14+12.6)/Math.log10(1.075))-55);
        double y = Math.tan(Math.toRadians(angleRight)) * x;
        return new double[] {x, y};
    }
    public void setPositionWithRobotPos(Pose2d robotPos) {
        double[] XandY = getDistanceFromCamera();
        double robotHeading = robotPos.heading.toDouble();
        double fieldCameraAngle = robotPos.heading.toDouble() + Math.toRadians(CameraVariables.cameraAngleFromRobotFront);
        targetsFieldX = robotPos.position.x + Math.cos(fieldCameraAngle)*XandY[0] + Math.sin(fieldCameraAngle) * -XandY[1] + Math.cos(robotHeading)*CameraVariables.cameraXFromRobotCenter + Math.sin(robotHeading)*-CameraVariables.cameraYFromRobotCenter;
        targetsFieldY = robotPos.position.y + Math.sin(fieldCameraAngle)*XandY[0] + Math.cos(fieldCameraAngle) * XandY[1] + Math.sin(robotHeading)*CameraVariables.cameraXFromRobotCenter + Math.cos(robotHeading)*-CameraVariables.cameraYFromRobotCenter;
    }
//    public void getRobotPosWithCamera(Pose2d robotPos) {
//        double[] XandY = getDistanceFromCamera();
//        double fieldCameraAngle = robotPos.heading.toDouble() + Math.toRadians(CameraVariables.cameraAngleFromRobotFront);
//        targetsFieldX = robotPos.position.x + Math.cos(fieldCameraAngle)*XandY[0] + Math.sin(fieldCameraAngle) * -XandY[1];
//        targetsFieldY = robotPos.position.y + Math.sin(fieldCameraAngle)*XandY[0] + Math.cos(fieldCameraAngle) * XandY[1];
//    }

    public double getFieldXPos() {
        return targetsFieldX;
    }
    public double getFieldYPos() {
        return targetsFieldY;
    }

    private Mat blur(Mat inputMat) {
        Mat outputMat = new Mat();
        Size size = new Size(blurLevel, blurLevel);
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
        Scalar lowerBound = new Scalar(lowerScalar1, lowerScalar2, lowerScalar3); // Lower boundary for the YCrCb color range
        Scalar upperBound = new Scalar(upperScalar1, upperScalar2, upperScalar3); // Upper boundary for the YCrCb color range

        // Apply the color range filter in the YCrCb color space
        Core.inRange(ycrcbFrame, lowerBound, upperBound, filteredFrame);


        return filteredFrame;
    }
    private Mat applyErosion(Mat inputMat) {
        Mat outputMat = new Mat();
        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeLevel, erodeLevel));
        Imgproc.erode(inputMat, outputMat, element);
        return outputMat;
    }
    public Mat applyDilation(Mat inputMat) {
        Mat outputMat = new Mat();
        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateLevel, dilateLevel));
        Imgproc.dilate(inputMat, outputMat, element);
        return outputMat;
    }
    public Mat addBlackToMat(Mat blackMat, Mat inputMat) {
        Mat outputMat = new Mat();
        inputMat.copyTo(outputMat, blackMat); // Copy the original frame using
        // the filter as a mask}
        return outputMat;
    }
    org.opencv.core.Rect getBlobsBoundingRect(Mat inputMat) {
        // Find contours in the filtered frame
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inputMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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

        // return the contour rect
        if (maxContour != null) {
            return Imgproc.boundingRect(maxContour);
        } else {
            return null;
        }
    }

    double[] getBlobsPixelPosition(org.opencv.core.Rect boundingRect){

        int distanceX = 0;
        int distanceY = 0;

        if (boundingRect != null) {
            int centerX = boundingRect.x + boundingRect.width / 2;
            int centerY = boundingRect.y + boundingRect.height; // / 2;

            // Calculate the x and y distances from the green dot (center of the screen) to
            // the red dot
            int screenCenterX = CameraVariables.cameraResolution.getHeight() / 2;
            int screenCenterY = CameraVariables.cameraResolution.getWidth() / 2;

            distanceX = centerX - screenCenterX;
            distanceY = centerY - screenCenterY;

        }
        return new double[]{distanceX, distanceY};

    }
    Mat getMatForDisplay(Mat mask, Mat blur, org.opencv.core.Rect boundingRect){

        int distanceX = 0;
        int distanceY = 0;

        Mat MatForDisplay = addBlackToMat(mask, blur);

        // Draw a red rectangle around the largest contour and a red dot in the center
        if (boundingRect != null) {

            Imgproc.rectangle(MatForDisplay, boundingRect.tl(), boundingRect.br(), new Scalar(0, 0, 255), 3);

            int centerX = boundingRect.x + boundingRect.width / 2;
            int centerY = boundingRect.y + boundingRect.height; // / 2;
            Imgproc.circle(MatForDisplay, new org.opencv.core.Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);

            // Calculate the x and y distances from the green dot (center of the screen) to
            // the red dot
            int screenCenterX = CameraVariables.cameraResolution.getHeight() / 2;
            int screenCenterY = CameraVariables.cameraResolution.getWidth() / 2;
            Imgproc.circle(MatForDisplay, new org.opencv.core.Point(screenCenterX, screenCenterY), 5, new Scalar(0, 255, 0), -1);
            distanceX = centerX - screenCenterX;
            distanceY = centerY - screenCenterY;
            String distanceInfo = "Distance: X=" + distanceX + ", Y=" + distanceY;
            Imgproc.putText(MatForDisplay, distanceInfo, new org.opencv.core.Point(20, 30), Imgproc.FONT_HERSHEY_SIMPLEX,
                    1, new Scalar(0, 255, 0), 3);


            // Add text information about the width and height of the rectangle below the x
            // and y dimensions
            String rectInfo = "Width: " + boundingRect.width + ", Height: " + boundingRect.height;
            Imgproc.putText(MatForDisplay, rectInfo, new org.opencv.core.Point(20, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 1,
                    new Scalar(0, 255, 255), 3);
        }

        return MatForDisplay;

    }
}

