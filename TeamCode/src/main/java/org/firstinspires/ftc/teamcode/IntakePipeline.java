package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import android.os.Environment;

class IntakePipeline extends OpenCvPipeline
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

    boolean saveMat = false;
    int pick =  1;

    public IntakePipeline(double fx, double fy, double cx, double cy)
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
    long currentTime = 0;
    long lastProcessingTime = 0;

    @Override
    public Mat processFrame(Mat input)
    {

//        if (input != null) {
//            if (saveMat) {
//                try {
//                    String lfilePath = String.format("%s/FIRST/savedData/tray_" + "Intake" + pick + ".jpg", Environment.getExternalStorageDirectory().getAbsolutePath());
//                    Imgcodecs.imwrite(lfilePath, input);
//                    pick++;
//                    saveMat = false;
//
//                } catch (Exception e) {
//
//                }
//            }
//        }

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

        return input;
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
    public void setSaveMat(boolean saveMatin) {
        saveMat = saveMatin;
    }
}