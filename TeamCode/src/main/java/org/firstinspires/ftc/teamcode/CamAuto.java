package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Auto Cam Test", group = "B")
public class CamAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initCameraProcessor();

        Pose2d initialPose = new Pose2d(6, 64, Math.PI*.75);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(6, 40), -Math.PI*.5);

        Action firstTrajectoryActionChosen = pathToRungs.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        firstTrajectoryActionChosen
                )
        );
    }
    private void initCameraProcessor() {
        List myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();

        BlobProcessor blob = new BlobProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        builder.setCamera(webcam1);

        builder.setCameraResolution(new Size(1280, 960));

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);}}


        // Create the AprilTag processor.
        /*AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
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

                .build();*/
/*
import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;CameraReader cameraReader = new CameraReader(1);


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        //builder.setLiveViewContainerId(0);
        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }

        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");


        builder.setCamera(webcam1);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 960));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(false);

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
        builder2.setCameraResolution(new Size(1280, 960));

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

    public static class CameraReader implements VisionProcessor {
        int id;
        CameraReader(){
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {

            CameraVariables.cameraStream1 = frame;


            if (CameraVariables.cameraStream1 != null){
                // This rotates the cameras
                Mat rotatedCameraStream1 = new Mat();

                Core.rotate(CameraVariables.cameraStream1, rotatedCameraStream1,Core.ROTATE_90_CLOCKWISE);


                // convert the mat to a bitmap before displaying it on the ftc dashboard
                Bitmap b = Bitmap.createBitmap(rotatedCameraStream1.width(), rotatedCameraStream1.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(rotatedCameraStream1, b);

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
}*/
