package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
public class FreezTesst extends LinearOpMode
{

    static final double FEET_PER_METER = 3.28084;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode()
    {
        PipelineLibrary pipelineLibrary = new PipelineLibrary(hardwareMap);

        waitForStart();

        pipelineLibrary.camera1.pauseViewport();
        pipelineLibrary.camera2.pauseViewport();
        pipelineLibrary.camera3.pauseViewport();

        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = pipelineLibrary.aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        pipelineLibrary.aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        pipelineLibrary.aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        telemetry.addData("", "");
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                    }
                }

                telemetry.addData("Cam1 FPS", pipelineLibrary.camera1.getFps());
                telemetry.addData("Cam1 Overhead ms", pipelineLibrary.camera1.getOverheadTimeMs());
                telemetry.addData("Cam1 Pipeline ms", pipelineLibrary.camera1.getPipelineTimeMs());
                telemetry.addData("", "");
                telemetry.addData("Cam2 FPS", pipelineLibrary.camera2.getFps());
                telemetry.addData("Cam2 Overhead ms", pipelineLibrary.camera2.getOverheadTimeMs());
                telemetry.addData("Cam2 Pipeline ms", pipelineLibrary.camera2.getPipelineTimeMs());
                telemetry.addData("", "");
                telemetry.addData("Cam3 FPS", pipelineLibrary.camera3.getFps());
                telemetry.addData("Cam3 Overhead ms", pipelineLibrary.camera3.getOverheadTimeMs());
                telemetry.addData("Cam3 Pipeline ms", pipelineLibrary.camera3.getPipelineTimeMs());

                if (gamepad1.aWasPressed()){
                    pipelineLibrary.intakePipeline.setSaveMat(true);
                }

                telemetry.addData("", "");
                int[] zoneColors = pipelineLibrary.trayPipeline.getBallColors();
                telemetry.addData("Zone 1", zoneColors[0]);
                telemetry.addData("Zone 2", zoneColors[1]);
                telemetry.addData("Zone 3", zoneColors[2]);
                telemetry.addData("Zone 4", zoneColors[3]);
                telemetry.addData("Zone 5", zoneColors[4]);

                telemetry.update();
            }

            sleep(20);
        }
    }
}