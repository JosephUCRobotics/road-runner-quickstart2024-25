package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@Disabled
@TeleOp(name="Tray Test")
public class TrayControllerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        TrayController trayController = new TrayController(hardwareMap);
        TwoCamLibrary camLibrary = new TwoCamLibrary(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.leftBumperWasPressed()) {
                    trayController.spinLeft();
                } else if (gamepad1.rightBumperWasPressed()) {
                    trayController.spinRight();
                }



//                if (((DcMotorEx) shoot_l).getVelocity() > 500) {
                if (gamepad1.y) {
                    trayController.setAutoSortTo(false);
                } else {
                    trayController.setAutoSortTo(true);
                }
                if (gamepad1.backWasPressed()){
                    trayController.resetTrayPosition();
                }

                if (gamepad1.bWasPressed()){
                    trayController.setZoneColors(new int[]{1, 1, 2, 0, 0});
                    trayController.setCode("PGP");
                }
                if (gamepad1.xWasPressed()){
                    trayController.setZoneColors(new int[]{1, 0, 1, 0, 0});
                    trayController.setCode("PGP");
                }

                if (gamepad1.aWasPressed()){
                    trayController.spinToNextInCode();
                }

                int[] zoneColors = camLibrary.trayProcessor.getBallColors();
                trayController.update();

//                telemetry.addData("Zone 1", zoneColors[0]);
//                telemetry.addData("Zone 2", zoneColors[1]);
//                telemetry.addData("Zone 3", zoneColors[2]);
//                telemetry.addData("Zone 4", zoneColors[3]);
//                telemetry.addData("Zone 5", zoneColors[4]);
//                telemetry.update();
            }
        }
    }
}