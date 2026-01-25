package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.cameraProcessor.TrayProcessor;
import org.firstinspires.ftc.teamcode.cameraProcessor.TrayProcessorV2;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
@Disabled

@TeleOp(name = "Three Camera Test")
public class ThreeCamTest extends LinearOpMode
{
    VisionPortal portal1;
    VisionPortal portal2;
    private WebcamName webcam2, webcam3;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;

    AprilTagProcessor aprilTagProcessor1;
    AprilTagProcessor aprilTagProcessor2;
    TrayProcessorV2 trayProcessor;

    private LED green;
    private LED green2;
    private LED green3;
    private LED green4;
    private LED red;
    private LED red2;
    private LED red3;
    private LED red4;

    @Override
    public void runOpMode() throws InterruptedException
    {
        green = hardwareMap.get(LED.class, "green");
        green2 = hardwareMap.get(LED.class, "green2");
        green3 = hardwareMap.get(LED.class, "green3");
        green4 = hardwareMap.get(LED.class, "green4");
        red = hardwareMap.get(LED.class, "red");
        red2 = hardwareMap.get(LED.class, "red2");
        red3 = hardwareMap.get(LED.class, "red3");
        red4 = hardwareMap.get(LED.class, "red4");

        green.enable(true);
        green2.enable(true);
        green3.enable(true);
        green4.enable(true);

        red.enable(true);
        red2.enable(true);
        red3.enable(true);
        red4.enable(true);

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcam3 = hardwareMap.get(WebcamName.class, "Webcam 3");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam2, webcam3);


        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.
        aprilTagProcessor1 = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();
        trayProcessor = new TrayProcessorV2();

        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().
        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(trayProcessor)
                .build();

        portal2 = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setLiveViewContainerId(portal2ViewId)
                .addProcessor(aprilTagProcessor2)
                .build();

        waitForStart();

        // Main Loop
        while (opModeIsActive())
        {
            trayProcessor.setSaveMat(gamepad1.y);
            doCameraSwitching();
            // Just show some basic telemetry to demonstrate both processors are working in parallel
            // on their respective cameras. If you want to see more detail about the information you
            // can get back from the processor, you should look at ConceptAprilTag.
//            telemetry.addData("red", trayProcessor.getRGB()[0]);
//            telemetry.addData("green", trayProcessor.getRGB()[1]);
//            telemetry.addData("blue", trayProcessor.getRGB()[2]);
//
//            telemetry.addData("L", trayProcessor.getLba()[0]);
//            telemetry.addData("b", trayProcessor.getLba()[1]);
//            telemetry.addData("a", trayProcessor.getLba()[2]);
//            String color = "";
//            if (trayProcessor.getRGB()[0]+trayProcessor.getRGB()[1]+trayProcessor.getRGB()[2] < 65){
//                color = "No Ball";
//            } else if (trayProcessor.getRGB()[1] > trayProcessor.getRGB()[2]) {
//                color = "Green Ball";
//            } else {
//                color = "Purple Ball";
//            }Ball
//            telemetry.addData("Ball Type", color);
            telemetry.addData("Number of tags in Camera 1", aprilTagProcessor1.getDetections().size());
            telemetry.addData("Number of tags in Camera 2", aprilTagProcessor2.getDetections().size());
            telemetry.addData("active camera", portal2.getActiveCamera());
            telemetry.update();
            sleep(20);
        }
    }

    private void doCameraSwitching() {
        if (portal2.getCameraState() == CameraState.STREAMING) {
            // If the left bumper is pressed, use Webcam 1.
            // If the right bumper is pressed, use Webcam 2.
            boolean newLeftBumper = gamepad1.left_bumper;
            boolean newRightBumper = gamepad1.right_bumper;
            if (newLeftBumper && !oldLeftBumper) {
                portal2.setActiveCamera(webcam2);
            } else if (newRightBumper && !oldRightBumper) {
                portal2.setActiveCamera(webcam3);
            }
            oldLeftBumper = newLeftBumper;
            oldRightBumper = newRightBumper;
        }

    }
}
