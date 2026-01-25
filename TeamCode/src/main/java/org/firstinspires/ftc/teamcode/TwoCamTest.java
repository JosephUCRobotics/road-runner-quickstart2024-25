package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
@TeleOp(name = "Two Camera Test", group = "C")
public class TwoCamTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException

    {
        TwoCamLibrary camLibrary = new TwoCamLibrary(hardwareMap);

        waitForStart();


        // Main Loop
        while (opModeIsActive()) {
            camLibrary.trayProcessor.setSaveMat(gamepad1.y, "test_");
            telemetry.addData("Number of tags in Camera 1", camLibrary.aprilTagProcessor.getDetections().size());
            telemetry.addData("Number of tags in Camera 2", camLibrary.aprilTagProcessor2.getDetections().size());
            telemetry.addData("", "");
            int[] zoneColors = camLibrary.trayProcessor.getBallColors();
            telemetry.addData("Zone 1", zoneColors[0]);
            telemetry.addData("Zone 2", zoneColors[1]);
            telemetry.addData("Zone 3", zoneColors[2]);
            telemetry.addData("Zone 4", zoneColors[3]);
            telemetry.addData("Zone 5", zoneColors[4]);
            telemetry.addData("", "");
            double[] intakeColors = camLibrary.trayProcessor.getp4_g4_p5_g5();
            telemetry.addData("p4", intakeColors[0]);
            telemetry.addData("g4", intakeColors[1]);
            telemetry.addData("p5", intakeColors[2]);
            telemetry.addData("g5", intakeColors[3]);
            telemetry.update();
            sleep(20);
        }
    }
}
