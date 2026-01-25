package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DashboardVariables.x;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Display_Heading")
@Disabled
public class Display_Heading extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.PI*.5);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        sleep(100);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive.updatePoseEstimate();
                telemetry.addData("Headings2", drive.localizer.getPose().heading.toDouble());
//                telemetry.addData("x", drive.localizer.getPose().position.x);
//                telemetry.addData("y", drive.localizer.getPose().position.y);
                telemetry.update();
            }
        }
    }
}
