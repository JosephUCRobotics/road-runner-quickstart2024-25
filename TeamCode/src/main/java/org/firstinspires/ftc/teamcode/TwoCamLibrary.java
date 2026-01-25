package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
public class TwoCamLibrary {
    VisionPortal portal1;
    VisionPortal portal2;

    AprilTagProcessor aprilTagProcessor;
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

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            3,12,15.25,0);
    //            -2.5, -7.25, 15.75, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -103, 180, 0);


    public TwoCamLibrary(HardwareMap hardwareMap) {
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
        int portal2ViewId = viewIds[1];


        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.
        aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor =  new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation).build();

        trayProcessor = new TrayProcessorV2();

        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().
        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(aprilTagProcessor2)
                .addProcessor(trayProcessor)
                .build();

        portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setLiveViewContainerId(portal2ViewId)
                .addProcessor(aprilTagProcessor)
                .build();

    }

}
