package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.cameraProcessor.ThrottledAprilTagProcessor;
import org.firstinspires.ftc.teamcode.cameraProcessor.TrayProcessor;
import org.firstinspires.ftc.teamcode.cameraProcessor.TrayProcessorV2;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;


/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
public class TwoCamLibrary {

    private ElapsedTime wateTimer = new ElapsedTime();
    private ElapsedTime exposureTimer = new ElapsedTime();
    private long portal1StallCheck = 0;
    private long portal2StallCheck = 0;

    float currentFPS1 = 0;
    float currentFPS2 = 0;
    VisionPortal portal1;
    VisionPortal portal2;

    ThrottledAprilTagProcessor aprilTagProcessor;
//    AprilTagProcessor aprilTagProcessor2;
    TrayProcessorV2 trayProcessor;
//
    private LED green;
    private LED green2;
    private LED green3;
    private LED green4;
    private LED red;
    private LED red2;
    private LED red3;
    private LED red4;

//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            3,12,15.25,0);
//    //            -2.5, -7.25, 15.75, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -103, 180, 0);

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            9.5,-2,12.5,0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -71.5, 2, 0);

    int portal2ViewId;
    HardwareMap hardwareMapb;

    public TwoCamLibrary(HardwareMap hardwareMap) {
        hardwareMapb = hardwareMap;
//        green = hardwareMap.get(LED.class, "green");
//        green2 = hardwareMap.get(LED.class, "green2");
//        green3 = hardwareMap.get(LED.class, "green3");
//        green4 = hardwareMap.get(LED.class, "green4");
//        red = hardwareMap.get(LED.class, "red");
//        red2 = hardwareMap.get(LED.class, "red2");
//        red3 = hardwareMap.get(LED.class, "red3");
//        red4 = hardwareMap.get(LED.class, "red4");
//
//        green.enable(true);
//        green2.enable(true);
//        green3.enable(true);
//        green4.enable(true);
//
//        red.enable(true);
//        red2.enable(true);
//        red3.enable(true);
//        red4.enable(true);

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        portal2ViewId = viewIds[1];

        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.
//        aprilTagProcessor2 = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(205.5, 205.5, 79.7, 60.2).build();
//        aprilTagProcessor =  new AprilTagProcessor.Builder()
////                .setLensIntrinsics(205.5, 205.5, 79.7, 60.2)
//                .setLensIntrinsics(781.7495236, 775.3727598, 361.8332991, 206.8412553)
//                .setCameraPose(cameraPosition, cameraOrientation).build();
//        aprilTagProcessor.setDecimation(2.0f);

            aprilTagProcessor = new ThrottledAprilTagProcessor.Builder()
                    .setMaxProcessRateHz(5.0)          // ← Throttle here (adjust 5–12 based on needs)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .build();

        trayProcessor = new TrayProcessorV2();

        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().
        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(160, 120))
                .setLiveViewContainerId(portal1ViewId)
//                .addProcessor(aprilTagProcessor2)
                .addProcessor(trayProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();
        exposureTimer.reset();

//        portal2 = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setLiveViewContainerId(portal2ViewId)
//                .addProcessor(aprilTagProcessor)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(false)
//                .build();
        portal2 = new VisionPortal.Builder()
                        .setCamera(hardwareMapb.get(WebcamName.class, "Webcam 2"))
                        .setLiveViewContainerId(portal2ViewId)
                        .addProcessor(aprilTagProcessor)
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .enableLiveView(false)
                        .build();

//        while (portal1.getCameraState() != CameraState.STREAMING || portal2.getCameraState() != CameraState.STREAMING) {
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
//
//        ExposureControl exposureControl = portal1.getCameraControl(ExposureControl.class);
//        exposureControl.setAePriority(false);
//
//        ExposureControl exposureControl2 = portal2.getCameraControl(ExposureControl.class);
//        exposureControl2.setAePriority(false);


    }
    boolean portal2Started = false;
    ExposureControl exposureControl = null;
    ExposureControl exposureControl2 = null;
    boolean exposureControlFailed = false;
    public boolean setExposure(){
//        if (portal1.getCameraState() == CameraState.STREAMING && !portal2Started){
//            if (wateTimer.milliseconds()  > 1500){
//                portal2 = new VisionPortal.Builder()
//                        .setCamera(hardwareMapb.get(WebcamName.class, "Webcam 2"))
//                        .setLiveViewContainerId(portal2ViewId)
//                        .addProcessor(aprilTagProcessor)
//                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                        .enableLiveView(false)
//                        .build();
//                portal2Started = true;
//            }
//        } else {
//            wateTimer.reset();
//        }

        if (exposureTimer.milliseconds() > 1000) {
            exposureControlFailed = false;
        }

        if (portal2 != null){
            if (/*portal1.getCameraState() == CameraState.STREAMING && */portal2.getCameraState() == CameraState.STREAMING && !exposureControlFailed) {
                boolean exposureSet = true;

//                if (exposureControl == null) {
//                    exposureControl = portal1.getCameraControl(ExposureControl.class);
//                }
                if (exposureControl2 == null) {
                    exposureControl2 = portal2.getCameraControl(ExposureControl.class);
                }

                if (exposureControl != null) {
                    exposureControl.setAePriority(false);
                } else {
                    exposureSet = false;
                    exposureControlFailed = true;
                    exposureTimer.reset();

                }


                if (exposureControl2 != null) {
                    exposureControl2.setAePriority(false);

                }else {
                    exposureSet = false;
                    exposureControlFailed = true;
                    exposureTimer.reset();
                }

                return exposureSet;

            } else {
                return false;
            }
        } else {
            return false;
        }

    }
//    public boolean camera1IsStreaming(){
//        return portal1.getCameraState() == CameraState.STREAMING;
//    }
    public boolean camera2IsStreaming(){

        if (portal2 != null){
            currentFPS2 = portal2.getFps();
            if (currentFPS2 > 0) {
                portal2StallCheck = System.currentTimeMillis();
                return portal2.getCameraState() == CameraState.STREAMING;
            } else if (System.currentTimeMillis() - portal2StallCheck > 1500) {
//                new Thread(() -> {
//                    try {
//                        portal2.close();
//                        portal1.close();
//                    } catch (Exception e){
//
//                    }
//                }).start();

                return false;
            } else {
                return portal2.getCameraState() == CameraState.STREAMING;
            }

        } else {
            return false;
        }
    }

//    public String checkForCamErrors(){
//        String status = "Ok";
//        currentFPS1 = portal1.getFps();
//        currentFPS2 = portal2.getFps();
//
//        if (currentFPS1 > 0) {
//            portal1StallCheck = System.currentTimeMillis();
//        } else if (System.currentTimeMillis() - portal1StallCheck > 1500) {
//            portal1.close();
//            status = "1 Not OK";
//        }
//
//        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//
//        if (currentFPS2 > 0 && detections != null) {
//            portal2StallCheck = System.currentTimeMillis();
//        } else if (System.currentTimeMillis() - portal2StallCheck > 1500) {
//            portal2.close();
//            status = "2 Not OK" + detections;
//        }

//        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//        if (detections != null){
//            lastS
//        }

//        String status = "Ok";
//        VisionPortal.CameraState state = portal1.getCameraState();
//
//        if (state == VisionPortal.CameraState.ERROR || state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED || state != CameraState.CAMERA_DEVICE_READY) {
//            // If the camera is failing, don't let it try to auto-restart
//            // This stops the native driver from looping and freezing the Hub
//            portal1.stopStreaming();
//            status = "Camera Disconnected 1 - Stopped Stream to prevent freeze";
//        }
//
//        VisionPortal.CameraState state2 = portal2.getCameraState();
//
//        if (state == VisionPortal.CameraState.ERROR || state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
//            // If the camera is failing, don't let it try to auto-restart
//            // This stops the native driver from looping and freezing the Hub
//            portal2.stopStreaming();
//            status = "Camera Disconnected 2 - Stopped Stream to prevent freeze";
//        }
//        return status;
//    }

    public class AprilTagTest extends AprilTagProcessor {

        @Override
        public void setDecimation(float decimation) {

        }

        @Override
        public void setPoseSolver(PoseSolver poseSolver) {

        }

        @Override
        public int getPerTagAvgPoseSolveTime() {
            return 0;
        }

        @Override
        public ArrayList<AprilTagDetection> getDetections() {
            return null;
        }

        @Override
        public ArrayList<AprilTagDetection> getFreshDetections() {
            return null;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        private long lastProcessTime = 0;
        private final long THROTTLE_MS = 200;

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            long currentTime = System.currentTimeMillis();

            if (currentTime - lastProcessTime >= THROTTLE_MS) {
                lastProcessTime = currentTime;
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
    }

}
