package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

class TrayPipeline extends OpenCvPipeline {
    Mat cameraMatrix;

    double fx, fy, cx, cy;

    // Region of Interest Rectangles
    Rect rect1 = new Rect(52, 60, 50, 60);
    Rect rect2 = new Rect(0, 82, 35, 38);
    Rect rect3 = new Rect(0, 10, 70, 55);
    Rect rect1H = new Rect(18, 85, 40, 35);
    Rect rect2H = new Rect(0, 28, 35, 55);
    Rect rect3H = new Rect(45, 25, 55, 50);
    Rect rect4 = new Rect(150, 70, 10, 50);
    Rect rect5 = new Rect(127, 30, 25, 30);

    int[] ballColors = {0, 0, 0, 0, 0};
    boolean trayIsHalfOff = false;

    // Reusable Mats to prevent memory churn
    Mat colorMat = new Mat();
    Mat aChannel = new Mat();
    Mat purpleThreshold = new Mat();
    Mat greenThreshold = new Mat();
    Mat mask = new Mat();

    Mat purpleOval1 = new Mat(), greenOval1 = new Mat();
    Mat purpleOval2 = new Mat(), greenOval2 = new Mat();
    Mat purpleOval3 = new Mat(), greenOval3 = new Mat();
    Mat purpleRect4 = new Mat(), greenRect4 = new Mat();
    Mat purpleRect5 = new Mat(), greenRect5 = new Mat();

    long zone1Size, zone2Size, zone3Size, zone4Size, zone5Size;
    double purplePixelPrecent1, greenPixelPrecent1, purplePixelPrecent2, greenPixelPrecent2;
    double purplePixelPrecent3, greenPixelPrecent3, purplePixelPrecent4, greenPixelPrecent4;
    double purplePixelPrecent5, greenPixelPrecent5;

    long lastProcessingTime = 0;

    public TrayPipeline(double fx, double fy, double cx, double cy) {
        this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
        constructMatrix();
    }

    @Override
    public Mat processFrame(Mat input) {
//        long currentTime = System.currentTimeMillis();
//        if (currentTime - lastProcessingTime < 100) return input;
//        lastProcessingTime = currentTime;

        // 1. Color Conversion
        Imgproc.cvtColor(input, colorMat, Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(colorMat, aChannel, 1);

        // 2. Thresholding
        Imgproc.threshold(aChannel, purpleThreshold, 145, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(aChannel, greenThreshold, 113, 255, Imgproc.THRESH_BINARY_INV);

        // 3. Zone Processing with Automatic Submat Release

        processZone(purpleThreshold, greenThreshold, trayIsHalfOff ? rect1H : rect1, purpleOval1, greenOval1, true);
        processZone(purpleThreshold, greenThreshold, trayIsHalfOff ? rect2H : rect2, purpleOval2, greenOval2, !trayIsHalfOff); // Use copyTo if not half off
        processZone(purpleThreshold, greenThreshold, trayIsHalfOff ? rect3H : rect3, purpleOval3, greenOval3, true);

        // Rect 4 and 5
        processZone(purpleThreshold, greenThreshold, rect4, purpleRect4, greenRect4, false); // Rects use copyTo
        processZone(purpleThreshold, greenThreshold, rect5, purpleRect5, greenRect5, true);  // Oval

        // 4. Analysis
        updatePercentages();
        calculateBallColors();

        return input;
    }

    /**
     * Helper to handle submat creation, processing, and mandatory release.
     */
    Mat pSub = new Mat();
    Mat gSub = new Mat();
    private void processZone(Mat purpSrc, Mat greenSrc, Rect roi, Mat purpDst, Mat greenDst, boolean asOval) {
        purpSrc.submat(roi).copyTo(pSub);
        greenSrc.submat(roi).copyTo(gSub);

        if (asOval) {
            convertToOvalMatInternal(pSub, purpDst);
            convertToOvalMatInternal(gSub, greenDst);
        } else {
            pSub.copyTo(purpDst);
            gSub.copyTo(greenDst);
        }

        // CRITICAL: Prevent the memory leak
        pSub.release();
        gSub.release();
    }

    private void convertToOvalMatInternal(Mat src, Mat dst) {
        mask.setTo(new Scalar(0));
        if (mask.rows() != src.rows() || mask.cols() != src.cols()) {
            mask.release();
            mask.create(src.size(), CvType.CV_8U);
        }

        Point center = new Point(src.cols() / 2.0, src.rows() / 2.0);
        Size axes = new Size(src.cols() / 2.0, src.rows() / 2.0);
        Imgproc.ellipse(mask, center, axes, 0, 0, 360, new Scalar(255), -1);

        dst.setTo(new Scalar(0));
        src.copyTo(dst, mask);
    }

    private void updatePercentages() {
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
    }

    private void calculateBallColors() {
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
    }

    void constructMatrix() {
        if (cameraMatrix != null) cameraMatrix.release();
        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    public int[] getBallColors() { return ballColors; }
    public void setTrayIsHalfOffTo(boolean halfOff) { trayIsHalfOff = halfOff; }

    @Override
    public void onViewportTapped() {
        // This is for if you stop the program but don't restart the robot
        // This is a good place to release everything if the pipeline is ever closed
        cameraMatrix.release();
        colorMat.release();
        aChannel.release();
        purpleThreshold.release();
        greenThreshold.release();
        mask.release();
        purpleOval1.release();
        purpleOval2.release();
        purpleOval3.release();
        purpleRect4.release();
        purpleRect5.release();
        greenOval1.release();
        greenOval2.release();
        greenOval3.release();
        greenRect4.release();
        greenRect5.release();
    }
}