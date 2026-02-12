package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

import java.util.ArrayList;

class TrayPipelineOld extends OpenCvPipeline
{
    private long nativeApriltagPtr;

    Mat cameraMatrix;

    double fx;
    double fy;
    double cx;
    double cy;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    //                  2
//                1   3
//                 4 5
//                intake
    Rect rect1 = new Rect(52,60,50,60);
    Rect rect2 = new Rect(0,82,35,38);
    Rect rect3 = new Rect(0,10,70,55);
    Rect rect1H = new Rect(18,85,40,35);
    Rect rect2H = new Rect(0,28,35,55);
    Rect rect3H = new Rect(45,25,55,50);
    Rect rect4 = new Rect(150,70,10,50);
    Rect rect5 = new Rect(127,30,25,30);
    int[] ballColors = {0,0,0,0,0};
    boolean trayIsHalfOff = false;

    public TrayPipelineOld(double fx, double fy, double cx, double cy)
    {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();
    }

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }
    Mat colorMat = new Mat();
    Mat aChannel = new Mat();
    Mat purpleThreshold = new Mat();
    Mat greenThreshold = new Mat();
    Mat purpleOval1 = new Mat();
    Mat greenOval1 = new Mat();
    Mat purpleOval2 = new Mat();
    Mat greenOval2 = new Mat();
    Mat purpleOval3 = new Mat();
    Mat greenOval3 = new Mat();
    Mat purpleRect4 = new Mat();
    Mat greenRect4 = new Mat();
    Mat purpleRect5 = new Mat();
    Mat greenRect5 = new Mat();
    long zone1Size = 0;
    long zone2Size = 0;
    long zone3Size = 0;
    long zone4Size = 0;
    long zone5Size = 0;
    double purplePixelPrecent1 = 0;
    double greenPixelPrecent1 = 0;
    double purplePixelPrecent2 = 0;
    double greenPixelPrecent2 = 0;
    double purplePixelPrecent3 = 0;
    double greenPixelPrecent3 = 0;
    double purplePixelPrecent4 = 0;
    double greenPixelPrecent4 = 0;
    double purplePixelPrecent5 = 0;
    double greenPixelPrecent5 = 0;
    long currentTime = 0;
    long lastProcessingTime = 0;
    @Override
    public Mat processFrame(Mat input)
    {

        currentTime = System.currentTimeMillis();

        // If 250ms haven't passed, just return the raw frame without processing
        if (currentTime - lastProcessingTime < 100) {
            return input;
        }

        lastProcessingTime = currentTime;

//        Was BGR2Lab in tray processor
        Imgproc.cvtColor(input, colorMat, Imgproc.COLOR_RGB2Lab);

        Core.extractChannel(colorMat, aChannel, 1);

        Imgproc.threshold(aChannel, purpleThreshold, 145, 255, Imgproc.THRESH_BINARY);

        Imgproc.threshold(aChannel, greenThreshold, 113, 255, Imgproc.THRESH_BINARY_INV);

        if (trayIsHalfOff) {
            convertToOvalMat(purpleThreshold.submat(rect1H), purpleOval1);
            convertToOvalMat(greenThreshold.submat(rect1H), greenOval1);
            convertToOvalMat(purpleThreshold.submat(rect2H), purpleOval2);
            convertToOvalMat(greenThreshold.submat(rect2H), greenOval2);
            convertToOvalMat(purpleThreshold.submat(rect3H), purpleOval3);
            convertToOvalMat(greenThreshold.submat(rect3H), greenOval3);
        } else {
            convertToOvalMat(purpleThreshold.submat(rect1), purpleOval1);
            convertToOvalMat(greenThreshold.submat(rect1), greenOval1);
            purpleThreshold.submat(rect2).copyTo(purpleOval2);
            greenThreshold.submat(rect2).copyTo(greenOval2);
            convertToOvalMat(purpleThreshold.submat(rect3), purpleOval3);
            convertToOvalMat(greenThreshold.submat(rect3), greenOval3);
        }
        purpleThreshold.submat(rect4).copyTo(purpleRect4);
        greenThreshold.submat(rect4).copyTo(greenRect4);
        convertToOvalMat(purpleThreshold.submat(rect5), purpleRect5);
        convertToOvalMat(greenThreshold.submat(rect5), greenRect5);

        zone1Size = purpleOval1.total();
        zone2Size = purpleOval2.total();
        zone3Size = purpleOval3.total();
        zone4Size = purpleRect4.total();
        zone5Size = purpleRect5.total();

        purplePixelPrecent1 = (double) Core.countNonZero(purpleOval1) / zone1Size;
        greenPixelPrecent1 = (double) Core.countNonZero(greenOval1) / zone1Size;
        purplePixelPrecent2 = (double) Core.countNonZero(purpleOval2) / zone2Size;
        greenPixelPrecent2 = (double) Core.countNonZero(greenOval2) / zone2Size;
        purplePixelPrecent3 = (double) Core.countNonZero(purpleOval3) / zone3Size;
        greenPixelPrecent3 = (double) Core.countNonZero(greenOval3) / zone3Size;
        purplePixelPrecent4 = (double) Core.countNonZero(purpleRect4) / zone4Size;
        greenPixelPrecent4 = (double) Core.countNonZero(greenRect4) / zone4Size;
        purplePixelPrecent5 = (double) Core.countNonZero(purpleRect5) / zone5Size;
        greenPixelPrecent5 = (double) Core.countNonZero(greenRect5) / zone5Size;

        if (purplePixelPrecent1 > .2 || greenPixelPrecent1 > .2) {
            if (purplePixelPrecent1 >= greenPixelPrecent1) {
                ballColors[0] = 1;
            } else {
                ballColors[0] = 2;
            }
        } else {
            ballColors[0] = 0;
        }

        if (purplePixelPrecent2 > .2 || greenPixelPrecent2 > .2) {
            if (purplePixelPrecent2 >= greenPixelPrecent2) {
                ballColors[1] = 1;
            } else {
                ballColors[1] = 2;
            }
        } else {
            ballColors[1] = 0;
        }

        if (purplePixelPrecent3 > .2 || greenPixelPrecent3 > .2) {
            if (purplePixelPrecent3 >= greenPixelPrecent3) {
                ballColors[2] = 1;
            } else {
                ballColors[2] = 2;
            }
        } else {
            ballColors[2] = 0;
        }

        if (purplePixelPrecent4 + purplePixelPrecent5 > .3 || greenPixelPrecent4 + greenPixelPrecent5 > .3) {
            if (purplePixelPrecent4 + purplePixelPrecent5 >= greenPixelPrecent4 + greenPixelPrecent5) {
                if (purplePixelPrecent4 > purplePixelPrecent5) {
                    ballColors[3] = 1;
                    ballColors[4] = 0;
                } else {
                    ballColors[3] = 0;
                    ballColors[4] = 1;
                }
            } else {
                if (greenPixelPrecent4 > greenPixelPrecent5) {
                    ballColors[3] = 2;
                    ballColors[4] = 0;
                } else {
                    ballColors[3] = 0;
                    ballColors[4] = 2;
                }
            }
        } else {
            ballColors[3] = 0;
            ballColors[4] = 0;
        }

        return input;
    }

    private Mat mask = new Mat();
    public void convertToOvalMat(Mat src, Mat dst) {
        // 1. Re-initialize the mask only if size/type changed
        mask.setTo(new Scalar(0));
        if (mask.rows() != src.rows() || mask.cols() != src.cols()) {
            mask.create(src.size(), CvType.CV_8U);
            mask.setTo(new Scalar(0));
        }

        // 2. Define the parameters for the ellipse
        Point center = new Point(src.cols() / 2.0, src.rows() / 2.0);
        Size axes = new Size(src.cols() / 2.0, src.rows() / 2.0);

        // Draw the filled white ellipse on the mask
        Imgproc.ellipse(mask, center, axes, 0, 0, 360, new Scalar(255), -1);

        // 3. Ensure dst is the right size/type without re-allocating if possible
        dst.setTo(new Scalar(0, 0, 0));

        // 4. Apply the mask
        src.copyTo(dst, mask);

        // IMPORTANT: No .release() here because we are REUSING these objects
    }

//    public static Mat convertToOvalMat(Mat src) {
//        // 1. Create a black mask with the same size as the source image
//        // The mask must be a single-channel (CV_8U) matrix.
//        Mat mask = new Mat(src.rows(), src.cols(), CvType.CV_8U, new Scalar(0));
//
//        // 2. Define the parameters for the ellipse
//        Point center = new Point(src.cols() / 2, src.rows() / 2);
//        // Axes lengths (major and minor radii)
//        Size axes = new Size(src.cols() / 2, src.rows() / 2);
//        double angle = 0.0; // Angle of rotation of the ellipse
//        double startAngle = 0.0;
//        double endAngle = 360.0; // 0 to 360 degrees for a full ellipse
//        Scalar color = new Scalar(255); // White color for the filled area
//        int thickness = -1; // -1 to fill the ellipse
//        int lineType = 8;
//        int shift = 0;
//
//        // Draw a filled white ellipse on the black mask
//        Imgproc.ellipse(mask, center, axes, angle, startAngle, endAngle, color, thickness, lineType, shift);
//
//        // 3. Create a destination Mat for the result, initialized to black
//        Mat result = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0, 0, 0));
//
//        // 4. Apply the mask: copy the source image pixels to the result image where the mask is white
//        src.copyTo(result, mask);
//
//        // Release the mask Mat as it's no longer needed
//        mask.release();
//
//        return result;
//    }

    public int[] getBallColors(){
        return ballColors;
    }
    public void setTrayIsHalfOffTo(boolean halfOff){
        trayIsHalfOff = halfOff;
    }


    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }
}