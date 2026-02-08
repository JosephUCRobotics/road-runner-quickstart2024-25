package org.firstinspires.ftc.teamcode.cameraProcessor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Environment;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.DashboardVariables;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class TrayProcessorV2 implements VisionProcessor {
//                  2
//                1   3
//                 4 5
//                intake
    Rect rect1 = new Rect(52,60,50,60);
    Rect rect2 = new Rect(0,82,35,38);
    Rect rect3 = new Rect(0,10,70,55);
    Rect rect4 = new Rect(150,70,10,50);
    Rect rect5 = new Rect(127,30,25,30);

    Mat displayMat;
    boolean saveMat = false;
    String fileNameClassifier = "";
    int pick =  1;
    double purplePixelCount = 0;

//    0 = no ball, 1 = purple ball, 2 = green ball
    int[] ballColors = {0,0,0,0,0};
    double[] p4_g4_p5_g5 = {0,0,0,0};


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }



    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if(frame != null) {
            if (saveMat) {
                try {
                    String lfilePath = String.format("%s/FIRST/savedData/tray_" + fileNameClassifier + pick + ".jpg", Environment.getExternalStorageDirectory().getAbsolutePath());
                    Imgcodecs.imwrite(lfilePath, frame);
                    pick++;
                    saveMat = false;

                } catch (Exception e) {

                }
            }

            Mat colorMat = new Mat();
            Imgproc.cvtColor(frame, colorMat, Imgproc.COLOR_BGR2Lab);

            //        Mat blurMat = new Mat();
            //        Imgproc.medianBlur(colorMat, blurMat, 11);

            Mat aChannel = new Mat();
            Core.extractChannel(colorMat, aChannel, 1);

            Mat purpleThreshold = new Mat();
            Imgproc.threshold(aChannel, purpleThreshold, 145, 255, Imgproc.THRESH_BINARY);

            Mat greenThreshold = new Mat();
            Imgproc.threshold(aChannel, greenThreshold, 113, 255, Imgproc.THRESH_BINARY_INV);

            //        Mat purpleErode = new Mat();
            //        Mat purpleErodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
            //        Imgproc.erode(purpleThreshold, purpleErode, purpleErodeKernel,new Point(-1, -1),5,Core.BORDER_CONSTANT, new Scalar(-1));
            //
            //        Mat greenErode = new Mat();
            //        Mat greenErodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
            //        Imgproc.erode(greenThreshold, greenErode, greenErodeKernel,new Point(-1, -1),5,Core.BORDER_CONSTANT, new Scalar(-1));
            //
            //        Mat purpleDilate = new Mat();
            //        Mat purpleDilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
            //        Imgproc.dilate(purpleErode, purpleDilate, purpleDilateKernel,new Point(-1, -1),5,Core.BORDER_CONSTANT, new Scalar(-1));
            //
            //        Mat greenDilate = new Mat();
            //        Mat greenDilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
            //        Imgproc.dilate(greenErode, greenDilate, greenDilateKernel,new Point(-1, -1),5,Core.BORDER_CONSTANT, new Scalar(-1));

            displayMat = greenThreshold;

            if (DashboardVariables.x + DashboardVariables.width > frame.cols() || DashboardVariables.y + DashboardVariables.height > frame.rows()) {
                rect1 = new Rect(1, 1, 10, 10);
            } else {
                rect1 = new Rect(DashboardVariables.x, DashboardVariables.y, DashboardVariables.width, DashboardVariables.height);
            }
            Mat purpleOval1 = convertToOvalMat(new Mat(purpleThreshold, rect1));
            Mat greenOval1 = convertToOvalMat(new Mat(greenThreshold, rect1));
            Mat purpleOval2 = new Mat(purpleThreshold, rect2);
            Mat greenOval2 = new Mat(greenThreshold, rect2);
            Mat purpleOval3 = convertToOvalMat(new Mat(purpleThreshold, rect3));
            Mat greenOval3 = convertToOvalMat(new Mat(greenThreshold, rect3));
            Mat purpleRect4 = new Mat(purpleThreshold, rect4);
            Mat greenRect4 = new Mat(greenThreshold, rect4);
            Mat purpleRect5 = convertToOvalMat(new Mat(purpleThreshold, rect5));
            Mat greenRect5 = convertToOvalMat(new Mat(greenThreshold, rect5));

            long zone1Size = purpleOval1.total();
            long zone2Size = purpleOval2.total();
            long zone3Size = purpleOval3.total();
            long zone4Size = purpleRect4.total();
            long zone5Size = purpleRect5.total();

            double purplePixelPrecent1 = (double) Core.countNonZero(purpleOval1) / zone1Size;
            double greenPixelPrecent1 = (double) Core.countNonZero(greenOval1) / zone1Size;
            double purplePixelPrecent2 = (double) Core.countNonZero(purpleOval2) / zone2Size;
            double greenPixelPrecent2 = (double) Core.countNonZero(greenOval2) / zone2Size;
            double purplePixelPrecent3 = (double) Core.countNonZero(purpleOval3) / zone3Size;
            double greenPixelPrecent3 = (double) Core.countNonZero(greenOval3) / zone3Size;
            double purplePixelPrecent4 = (double) Core.countNonZero(purpleRect4) / zone4Size;
            double greenPixelPrecent4 = (double) Core.countNonZero(greenRect4) / zone4Size;
            double purplePixelPrecent5 = (double) Core.countNonZero(purpleRect5) / zone5Size;
            double greenPixelPrecent5 = (double) Core.countNonZero(greenRect5) / zone5Size;
            p4_g4_p5_g5 = new double[]{purplePixelPrecent4, greenPixelPrecent4, purplePixelPrecent5, greenPixelPrecent5};

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


        return null;
    }
    public double[] getp4_g4_p5_g5(){
        return p4_g4_p5_g5;
    }
    public int[] getBallColors(){
        return ballColors;
    }
    public void setSaveMat(boolean saveMatin) {
        saveMat = saveMatin;
    }
    public void setSaveMat(boolean saveMatin, String name) {
        saveMat = saveMatin;
        fileNameClassifier = name;
    }


    public static Mat convertToOvalMat(Mat src) {
        // 1. Create a black mask with the same size as the source image
        // The mask must be a single-channel (CV_8U) matrix.
        Mat mask = new Mat(src.rows(), src.cols(), CvType.CV_8U, new Scalar(0));

        // 2. Define the parameters for the ellipse
        Point center = new Point(src.cols() / 2, src.rows() / 2);
        // Axes lengths (major and minor radii)
        Size axes = new Size(src.cols() / 2, src.rows() / 2);
        double angle = 0.0; // Angle of rotation of the ellipse
        double startAngle = 0.0;
        double endAngle = 360.0; // 0 to 360 degrees for a full ellipse
        Scalar color = new Scalar(255); // White color for the filled area
        int thickness = -1; // -1 to fill the ellipse
        int lineType = 8;
        int shift = 0;

        // Draw a filled white ellipse on the black mask
        Imgproc.ellipse(mask, center, axes, angle, startAngle, endAngle, color, thickness, lineType, shift);

        // 3. Create a destination Mat for the result, initialized to black
        Mat result = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0, 0, 0));

        // 4. Apply the mask: copy the source image pixels to the result image where the mask is white
        src.copyTo(result, mask);

        // Release the mask Mat as it's no longer needed
        mask.release();

        return result;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5);


//        if (displayMat != null && displayMat.cols() > 0) {
//            Bitmap bitmap = Bitmap.createBitmap(displayMat.cols(), displayMat.rows(), Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(displayMat, bitmap);
//
//            canvas.drawBitmap(bitmap, 0, 0, null);
//        }

        canvas.drawOval(rect1.x*4, rect1.y*4,
                rect1.x *4 + rect1.width*4,
                rect1.y*4 + rect1.height*4, paint);
        paint.setColor(Color.RED);
        canvas.drawRect(rect1.x*4, rect1.y*4,
                rect1.x *4+ rect1.width*4,
                rect1.y *4 + rect1.height*4, paint);

    }
}