package org.firstinspires.ftc.teamcode.cameraProcessor;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CameraVariables;
import org.firstinspires.ftc.teamcode.DashboardVariables;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class TrayProcessor implements VisionProcessor {
    Rect rect = new Rect(1,1,1,1);
    Rect rect2 = new Rect(1,1,1,1);
    Rect rect3 = new Rect(1,1,1,1);

    Scalar meanScalar;
    Scalar meanScalarLba;
    int Ball1Color = 0;
    int Ball2Color = 0;
    int Ball3Color = 0;


    Mat hist = new Mat();

    MatOfInt channels = new MatOfInt(0); // Channel 0 for grayscale
    Mat mask = new Mat(); // No mask
    MatOfInt histSize = new MatOfInt(256); // 256 bins
    MatOfFloat ranges = new MatOfFloat(0f, 256f); // Pixel value range 0-255
    Mat hist2 = new Mat();

    MatOfInt channels2 = new MatOfInt(0); // Channel 0 for grayscale
    Mat mask2 = new Mat(); // No mask
    MatOfInt histSize2 = new MatOfInt(256); // 256 bins
    MatOfFloat ranges2 = new MatOfFloat(0f, 256f); // Pixel value range 0-255
    Mat hist3 = new Mat();

    MatOfInt channels3 = new MatOfInt(0); // Channel 0 for grayscale
    Mat mask3 = new Mat(); // No mask
    MatOfInt histSize3 = new MatOfInt(256); // 256 bins
    MatOfFloat ranges3 = new MatOfFloat(0f, 256f); // Pixel value range 0-255

    int hist_w = 512;
    int hist_h = 400;
    int bin_w = (int) Math.round((double) hist_w / histSize.get(0, 0)[0]);

    Mat resultImage;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (DashboardVariables.x + DashboardVariables.width > frame.cols() || DashboardVariables.y + DashboardVariables.height > frame.rows()) {
            rect = new Rect(1, 1, 10, 10);
        } else {
            rect = new Rect(DashboardVariables.x, DashboardVariables.y, DashboardVariables.width, DashboardVariables.height);
        }



        // 1. Convert the image from BGR to Lab color space
        Mat labImage = new Mat();
        Imgproc.cvtColor(frame, labImage, Imgproc.COLOR_BGR2Lab);

        // 2. Split the channels (L, a, b)
        List<Mat> labChannels = new ArrayList<>();
        Core.split(labImage, labChannels);
        Mat luminanceChannel = labChannels.get(0);

        // 3. Apply CLAHE to the luminance channel
        // Create CLAHE object (adjust clipLimit and tileSize as needed)
        CLAHE clahe = Imgproc.createCLAHE();
        clahe.setClipLimit(4.0); // Default is 40, but a lower value might work better depending on image
//        clahe.setTileGridSize(new org.opencv.core.Size(8, 8)); // Default is 8x8
        clahe.apply(luminanceChannel, luminanceChannel);

        labChannels.set(0, luminanceChannel);
        // 4. Merge the equalized luminance channel back with the color channels
        Core.merge(labChannels, labImage);


        meanScalarLba = Core.mean(labImage);
        // 5. Convert the image back from Lab to BGR
        resultImage = new Mat();
        Imgproc.cvtColor(labImage, resultImage, Imgproc.COLOR_Lab2BGR);


        Mat subMat = convertToOvalMat(new Mat(resultImage, rect));


//        List<Mat> channelsMats = new ArrayList<>();
//        Core.split(subMat, channelsMats);
//
//        Mat grayImage = new Mat();
//        Imgproc.cvtColor(subMat, grayImage, Imgproc.COLOR_BGR2GRAY);
//
//        // Prepare for calcHist
////        List<Mat> images = new ArrayList<>();
////        images.add(grayImage);
//
//        // Prepare for calcHist
//        List<Mat> images = new ArrayList<>();
//        images.add(channelsMats.get(1));
//
//        // Calculate the histogram
//        Imgproc.calcHist(images, channels, mask, hist, histSize, ranges);
//
//
//
//
//
//
//        // Prepare for calcHist
//        List<Mat> images2 = new ArrayList<>();
//        images2.add(channelsMats.get(2));
//
//        // Calculate the histogram
//        Imgproc.calcHist(images2, channels2, mask2, hist2, histSize2, ranges2);
//
//
//
//
//
//        // Prepare for calcHist
//        List<Mat> images3 = new ArrayList<>();
//        images3.add(channelsMats.get(0));
//
//        // Calculate the histogram
//        Imgproc.calcHist(images3, channels3, mask3, hist3, histSize3, ranges3);


        // Calculate the mean (average) color of the sub-matrix
        meanScalar = Core.mean(subMat);

        if (meanScalar.val[2]+meanScalar.val[1]+meanScalar.val[0] < 90){
            Ball1Color = 0;
        } else if (meanScalar.val[1] > meanScalar.val[0]) {
            Ball1Color = 2;
        } else {
            Ball1Color = 1;
        }



        if (DashboardVariables.x2 + DashboardVariables.width2 > frame.cols() || DashboardVariables.y2 + DashboardVariables.height2 > frame.rows()) {
            rect2 = new Rect(1, 1, 10, 10);
        } else {
            rect2 = new Rect(DashboardVariables.x2, DashboardVariables.y2, DashboardVariables.width2, DashboardVariables.height2);
        }


        Mat subMat2 = convertToOvalMat(new Mat(frame, rect2));

        // Calculate the mean (average) color of the sub-matrix
        Scalar meanScalar2 = Core.mean(subMat2);

        if (meanScalar2.val[2]+meanScalar2.val[1]+meanScalar2.val[0] < 90){
            Ball2Color = 0;
        } else if (meanScalar2.val[1] > meanScalar2.val[0]) {
            Ball2Color = 2;
        } else {
            Ball2Color = 1;
        }




        if (DashboardVariables.x3 + DashboardVariables.width3 > frame.cols() || DashboardVariables.y3 + DashboardVariables.height3 > frame.rows()) {
            rect3 = new Rect(1, 1, 10, 10);
        } else {
            rect3 = new Rect(DashboardVariables.x3, DashboardVariables.y3, DashboardVariables.width3, DashboardVariables.height3);
        }


        Mat subMat3 = convertToOvalMat(new Mat(frame, rect3));

        // Calculate the mean (average) color of the sub-matrix
        Scalar meanScalar3 = Core.mean(subMat3);

        if (Math.abs(meanScalar3.val[1] - meanScalar3.val[0]) < 6){
            Ball3Color = 0;
        } else if (meanScalar3.val[1] > meanScalar3.val[0]) {
            Ball3Color = 3;
        } else {
            Ball3Color = 1;
        }
        return null;
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

    public double[] getRGB(){
        if (meanScalar != null && meanScalar.val[2] != 0){
            return new double[]{meanScalar.val[2], meanScalar.val[1], meanScalar.val[0]};
        } else {
            return new double[]{0, 0, 0};
        }

    }
    public double[] getLba(){
        if (meanScalarLba != null && meanScalarLba.val[2] != 0){
            return new double[]{meanScalarLba.val[0], meanScalarLba.val[1], meanScalarLba.val[2]};
        } else {
            return new double[]{0, 0, 0};
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5);


        if (resultImage != null && resultImage.cols()>0){
            Bitmap bitmap = Bitmap.createBitmap(resultImage.cols(), resultImage.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(resultImage, bitmap);

            canvas.drawBitmap(bitmap, 0, 0, null);
        }



//        paint.setColor(Color.RED);
//        paint.setStrokeWidth(1);
//        // Draw the histogram
//        for (int i = 1; i < histSize.get(0, 0)[0]; i++) {
//            canvas.drawLine(bin_w * (i - 1),hist_h - Math.round(hist.get(i - 1, 0)[0]*.1),
//                    bin_w * (i), hist_h - Math.round(hist.get(i, 0)[0]*.1), paint);
////            Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist.get(i - 1, 0)[0])),
////                    new Point(bin_w * (i), hist_h - Math.round(hist.get(i, 0)[0])),
////                    new Scalar(255, 255, 255), 2, 8, 0);
//        }
//
//        paint.setColor(Color.GREEN);
//        // Draw the histogram
//        for (int i = 1; i < histSize2.get(0, 0)[0]; i++) {
//            canvas.drawLine(bin_w * (i - 1),hist_h - Math.round(hist2.get(i - 1, 0)[0]*.1),
//                    bin_w * (i), hist_h - Math.round(hist2.get(i, 0)[0]*.1), paint);
////            Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist.get(i - 1, 0)[0])),
////                    new Point(bin_w * (i), hist_h - Math.round(hist.get(i, 0)[0])),
////                    new Scalar(255, 255, 255), 2, 8, 0);
//        }
//
//        paint.setColor(Color.BLUE);
//        // Draw the histogram
//        for (int i = 1; i < histSize3.get(0, 0)[0]; i++) {
//            canvas.drawLine(bin_w * (i - 1),hist_h - Math.round(hist3.get(i - 1, 0)[0]*.1),
//                    bin_w * (i), hist_h - Math.round(hist3.get(i, 0)[0]*.1), paint);
////            Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist.get(i - 1, 0)[0])),
////                    new Point(bin_w * (i), hist_h - Math.round(hist.get(i, 0)[0])),
////                    new Scalar(255, 255, 255), 2, 8, 0);
//        }
//
//        paint.setStrokeWidth(5);


        if(Ball1Color == 0){
            paint.setColor(Color.RED);
        } else if (Ball1Color == 1){
            paint.setColor(Color.BLUE);
        } else {
            paint.setColor(Color.GREEN);
        }

        canvas.drawOval(rect.x, rect.y,
                rect.x + rect.width,
                rect.y + rect.height, paint);




        if(Ball2Color == 0){
            paint.setColor(Color.RED);
        } else if (Ball2Color == 1){
            paint.setColor(Color.BLUE);
        } else {
            paint.setColor(Color.GREEN);
        }

        canvas.drawOval(rect2.x, rect2.y,
                rect2.x + rect2.width,
                rect2.y + rect2.height, paint);





        if(Ball3Color == 0){
            paint.setColor(Color.RED);
        } else if (Ball3Color == 1){
            paint.setColor(Color.BLUE);
        } else {
            paint.setColor(Color.GREEN);
        }

        canvas.drawOval(rect3.x, rect3.y,
                rect3.x + rect3.width,
                rect3.y + rect3.height, paint);
    }
}