package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


public class PipelineLibrary {
    OpenCvWebcam camera1;
    OpenCvWebcam camera2;
    OpenCvWebcam camera3; // Intake Camera
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    TrayPipeline trayPipeline;

    IntakePipeline intakePipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public PipelineLibrary(HardwareMap hardwareMap)
    {
        // Get the view ID for the monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Split the viewport so you can see both cameras on the screen side-by-side
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(
                cameraMonitorViewId,
                3, // Number of viewports
                OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); // Split screen vertically

        // Initialize Camera 2 (AprilTags)
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, 781.7495236, 775.3727598, 361.8332991, 206.8412553);
        camera2.setPipeline(aprilTagDetectionPipeline);

        // Initialize Camera 1 (Tray)
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);
        trayPipeline = new TrayPipeline(205.5, 205.5, 79.7, 60.2);
        camera1.setPipeline(trayPipeline);

        // Initialize Camera 3 (Intake)
        camera3 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 3"), viewportContainerIds[2]);
        intakePipeline = new IntakePipeline(205.5, 205.5, 79.7, 60.2);
        camera3.setPipeline(intakePipeline);

        // Open Camera 1
        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camera1.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG); }
            @Override public void onError(int errorCode) {
                if (camera1 != null) {
                    camera1.stopStreaming();
                    camera1.closeCameraDeviceAsync(() -> {});
                }
            }
        });

        // Open Camera 2
        camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camera2.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG); }
            @Override public void onError(int errorCode) {
                if (camera2 != null) {
                    camera2.stopStreaming();
                    camera2.closeCameraDeviceAsync(() -> {});
                }
            }
        });

        // Open Camera 3
        camera3.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camera3.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG); }
            @Override public void onError(int errorCode) {
                if (camera3 != null) {
                    camera3.stopStreaming();
                    camera3.closeCameraDeviceAsync(() -> {});
                }
            }
        });


    }
}