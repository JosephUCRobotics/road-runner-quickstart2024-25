package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.Environment;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.teamcode.cameraProcessor.BlobProcessor;
import org.firstinspires.ftc.teamcode.cameraProcessor.EdgeProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;


public class MainVision {
    static int webcam;
    public MainVision() throws InterruptedException {
        initCameraProcessor();
        getCameraSetting();
        setManualExposure(CameraVariables.exposure, CameraVariables.gain);

        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) new CameraDisplaySource(), 15);


    }
    private void initCameraProcessor() {
        // find the view id for both cameras
        List myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        int Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();


        // ToDo check if aprilTag and aprilTag2 are needed or if i can just use one.
        // Create Processors
//        CameraVariables.blob = new BlobProcessor();
        CameraVariables.edge = new EdgeProcessor();


        // Create the AprilTag processor.
        CameraVariables.aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1414.88, 1414.88, 709.08, 352.476)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        CameraVariables.aprilTag2 = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1414.88, 1414.88, 709.08, 352.476)
                // ... these parameters are fx, fy, cx, cy.

                .build();

//        CameraVariables.blob = new BlobProcessor();
        CameraVariables.edge = new EdgeProcessor();


        CameraReader cameraReader = new CameraReader(1);


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        CameraVariables.webcam1 = CameraVariables.hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraVariables.webcam2 = CameraVariables.hardwareMap.get(WebcamName.class, "Webcam 2");

        builder.setCamera(CameraVariables.webcam1);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(CameraVariables.cameraResolution);

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(false);


        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(CameraVariables.aprilTag2);
        builder.addProcessor(cameraReader);

        // Build the Vision Portal, using the above settings.
        builder.setLiveViewContainerId(Portal_1_View_ID);
        CameraVariables.visionPortal = builder.build();

        CameraVariables.visionPortal.setProcessorEnabled(CameraVariables.aprilTag2, false);
        CameraVariables.visionPortal.setProcessorEnabled(cameraReader, true);




        CameraReader cameraReader2 = new CameraReader(2);

        VisionPortal.Builder builder2 = new VisionPortal.Builder();

        builder2.setCamera(CameraVariables.webcam2);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder2.setCameraResolution(CameraVariables.cameraResolution);
        builder2.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder2.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder2.addProcessor(CameraVariables.aprilTag);
        builder2.addProcessor(cameraReader2);

        // Build the Vision Portal, using the above settings.
        builder2.setLiveViewContainerId(Portal_2_View_ID);
        CameraVariables.visionPortal2 = builder2.build();

        CameraVariables.visionPortal2.setProcessorEnabled(CameraVariables.aprilTag, false);
        CameraVariables.visionPortal2.setProcessorEnabled(cameraReader2, true);

    }   // end method initAprilTag()
    public static Vector<Double> getCameraPosWithAprilTag(AprilTagDetection detection) {
        double cx = detection.ftcPose.x;
        double cy = detection.ftcPose.y;
        // cx & cy are in inches

        double yaw = detection.ftcPose.yaw;
        // yaw is in digress

        double range = detection.ftcPose.range;
        // range is in inches

        double thetaA = 180;
        if (detection.id <= 6) thetaA = 0;
        // thetaA is in digress

        // the reason thees functions don't look right is because fieldPosition is rotated 90 digress from how we want it
        double ax = -detection.metadata.fieldPosition.get(1);
        double ay = detection.metadata.fieldPosition.get(0);
        // ax & ay are in inches

        double thetaCF = thetaA - yaw;
        // thetaCF is in digress

        if (cx == 0) {
            cx = 0.0000001;
        }

        double alpha = Math.atan(cy / cx);
        // alpha is in radians

        if (alpha < 0) {
            alpha += Math.PI;
        }

        double b = Math.toRadians(thetaCF) + alpha;
        // b is in radians

        double cxf = range * Math.cos(b);
        double cyf = range * Math.sin(b);
        // cxf & cyf are in inches


        //35.5, -70.5
        double by = ay - cyf;
        double bx = ax - cxf;
        // by & bx are in inches

        Vector<Double> output = new Vector<>();
        output.add(thetaCF);
        output.add(bx);
        output.add(by);
        output.add(ax);
        output.add(ay);
        output.add(thetaA);
        return output;
    }
    public static Vector<Double> getRobotPosFromCameraStream(double thetaCF, double by, double bx) {
        //
        //                                                          degrees
        double rx;
        double ry;
        double thetaCR;
        if (webcam == 2) {
            rx = 6;
            ry = 2;
            thetaCR = 131;
        } else {
            rx = 10;
            ry = 0;
            thetaCR = -33;
        }
//        double rx = -4;
//        double ry = 2;
//        double thetaCR = 45;
        // degrees
        double thetaRF = thetaCF - thetaCR;
        // degrees
        double range = Math.sqrt(Math.pow(ry, 2) + Math.pow(rx, 2));
        if (rx == 0) rx = 0.0000001;
        double alpha = Math.atan(ry / rx);
        // radians
        if (rx < 0) {
            if (ry > 0) {
                alpha += Math.PI;
            } else {
                alpha -= Math.PI;
            }
        }
        double b = Math.toRadians(thetaRF) + alpha;
        // radians
        double ryf = range * Math.sin(b);
        double rxf = range * Math.cos(b);
        double cy = by - ryf;
        double cx = bx - rxf;
        //thetaRF  += 90; // this is to change ware the zero is from up to right  ->

        Vector<Double> output = new Vector<>();
        output.add(thetaRF);
        output.add(cx);
        output.add(cy);
        return output;
    }
    private void getCameraSetting() throws InterruptedException {
        // Ensure Vision Portal has been setup.
        if (CameraVariables.visionPortal == null || CameraVariables.visionPortal2 == null) {
            return;
        }

        // Wait for the camera to be open
        if (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }
        }

        ExposureControl exposureControl = CameraVariables.visionPortal.getCameraControl(ExposureControl.class);
        CameraVariables.minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        CameraVariables.maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = CameraVariables.visionPortal.getCameraControl(GainControl.class);
        CameraVariables.minGain = gainControl.getMinGain();
        CameraVariables.maxGain = gainControl.getMaxGain();
    }
    protected static boolean setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Ensure Vision Portal has been setup.
        if (CameraVariables.visionPortal == null || CameraVariables.visionPortal2 == null) {
            return false;
        }

        // Wait for the camera to be open
        if (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }
        }

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = CameraVariables.visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        ExposureControl exposureControl2 = CameraVariables.visionPortal2.getCameraControl(ExposureControl.class);
        if (exposureControl2.getMode() != ExposureControl.Mode.Manual) {
            exposureControl2.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);
        exposureControl2.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);

        // Set Gain.
        GainControl gainControl = CameraVariables.visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        Thread.sleep(20);

        GainControl gainControl2 = CameraVariables.visionPortal2.getCameraControl(GainControl.class);
        gainControl2.setGain(gain);
        Thread.sleep(20);
        return (true);

    }
    public static class CameraDisplaySource implements CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            if (CameraVariables.lastBitmap != null){
                lastFrame.set(CameraVariables.lastBitmap);
                continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
            }
        }
    }
    public static class CameraReader implements VisionProcessor {
        int id;
        CameraReader(int idIn){
            id = idIn;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat rotatedCameraStream = new Mat();
            switch (CameraVariables.cameraLEDDirection) {
                case "top":
                    Core.rotate(frame, rotatedCameraStream, Core.ROTATE_90_COUNTERCLOCKWISE);
                    break;
                case "right":
                    Core.rotate(frame, rotatedCameraStream, Core.ROTATE_180);
                    break;
                case "bottom":
                    Core.rotate(frame, rotatedCameraStream, Core.ROTATE_90_CLOCKWISE);
                    break;
            }

            if(id == 1){
                CameraVariables.cameraStream1 = rotatedCameraStream;
            } else {
                CameraVariables.cameraStream2 = rotatedCameraStream;
            }

            if (CameraVariables.cameraStream1 != null && CameraVariables.cameraStream2 != null && id == 1){
                // This rotates the cameras

                // process frames separately hear
                Mat[] separatelyProcessedMats = MainVision.processMatsSeparately(CameraVariables.cameraStream1, CameraVariables.cameraStream2);

                // This joins the cameras in to one mat
                Mat dst = new Mat();
                List<Mat> src = Arrays.asList(separatelyProcessedMats[0], separatelyProcessedMats[1]);
                Core.hconcat(src, dst);

                // process frames as one hear

                Mat matForDisplay = MainVision.processMatsAsOne(dst);

                if (CameraVariables.matForDisplay != null) {
                    matForDisplay = CameraVariables.matForDisplay;
                }


                // convert the mat to a bitmap before displaying it on the ftc dashboard
                Bitmap b = Bitmap.createBitmap(matForDisplay.width(), matForDisplay.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(matForDisplay, b);

                CameraVariables.lastBitmap = b;
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }
    }
    protected static Mat[] processMatsSeparately(Mat mat1, Mat mat2){
//        mat1 = CameraVariables.blob.processFrame(mat1, 1);
//        mat2 = CameraVariables.blob.processFrame(mat2, 2);
        Mat notMat1 = CameraVariables.edge.processFrame(mat1, 1);

        return new Mat[]{mat1,mat2};
    }
    protected static Mat processMatsAsOne(Mat combinedMat){
//        combinedMat = CameraVariables.blob.processFrame(combinedMat);
        return combinedMat;
    }

    public class UpdatePositionWithCameras implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            return false;
        }
    }

    public Action updatePositionWithCameras() {
        return new UpdatePositionWithCameras();
    }

    public String saveMatsAsJPG(String takeName){
        try {
            String lfilePath = String.format("%s/FIRST/savedData/leftMat_"+ takeName+".jpg", Environment.getExternalStorageDirectory().getAbsolutePath());
            String rfilePath = String.format("%s/FIRST/savedData/rightMat_"+ takeName+".jpg", Environment.getExternalStorageDirectory().getAbsolutePath());
            Imgcodecs.imwrite(rfilePath, CameraVariables.cameraStream1);
            Imgcodecs.imwrite(lfilePath, CameraVariables.cameraStream2);

            return rfilePath;
        } catch (Exception e) {
            return e.getMessage();
        }
    }
}
