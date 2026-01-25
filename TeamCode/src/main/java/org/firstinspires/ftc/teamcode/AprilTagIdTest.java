package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Driver Oriented")
@Disabled
public class AprilTagIdTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MainVision vision = new MainVision();

        waitForStart();
        if (opModeIsActive()) {

        }
    }
}
