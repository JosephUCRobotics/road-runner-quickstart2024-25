package org.firstinspires.ftc.teamcode;

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
    public Mat processFrame(Mat frame) {
        Mat blur = blur(frame);
        mask = applyColorFilter(blur);
        mask = applyErosion(mask);
        mask = applyDilation(mask);
        displayed = addBlackToMat(mask, blur);
        updateBlobPos();
        return displayed;
    }
    private Mat blur(Mat inputMat) {
        int blurLevel = 20;
        Mat outputMat = new Mat();
        Size size = new Size(blurLevel, blurLevel);
        Imgproc.blur(inputMat, outputMat, size);
        return outputMat;
    }
    int lowerScalar1 = 10;
    int lowerScalar2 = 100;
    int lowerScalar3 = 135;
    int upperScalar1 = 115;
    int upperScalar2 = 125;
    int upperScalar3 = 200;
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

    int erodeLevel = 20;
    int dilateLevel = 50;
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

    int cameraWidth = 1280;
    int cameraHeight = 720;
    void updateBlobPos(){
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
            Imgproc.rectangle(displayed, boundingRect.tl(), boundingRect.br(), new Scalar(0, 0, 255), 3);

            int centerX = boundingRect.x + boundingRect.width / 2;
            int centerY = boundingRect.y + boundingRect.height / 2;
            Imgproc.circle(displayed, new org.opencv.core.Point(centerX, centerY), 5, new Scalar(0, 0, 255), -1);

            // Calculate the x and y distances from the green dot (center of the screen) to
            // the red dot
            int screenCenterX = cameraHeight*2 / 2;
            int screenCenterY = cameraWidth / 2;
            int distanceX = centerX - screenCenterX;
            int distanceY = centerY - screenCenterY;
            String distanceInfo = "Distance: X=" + distanceX + ", Y=" + distanceY;
            Imgproc.putText(displayed, distanceInfo, new org.opencv.core.Point(20, 75), Imgproc.FONT_HERSHEY_SIMPLEX,
                    2, new Scalar(0, 255, 0), 3);

            // Add text information about the width and height of the rectangle below the x
            // and y dimensions
            String rectInfo = "Width: " + boundingRect.width + ", Height: " + boundingRect.height;
            Imgproc.putText(displayed, rectInfo, new org.opencv.core.Point(20, 150), Imgproc.FONT_HERSHEY_SIMPLEX, 2,
                    new Scalar(0, 0, 255), 3);
        }
    }
}
