package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.opencv.core.Mat;

@Disabled
public final class SplineTest extends LinearOpMode {
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, beginPose);
//            drive.localizer.setPose(beginPose);

//            OTOSLocalizer otos = new OTOSLocalizer(hardwareMap, beginPose);
            telemetry.addData("heading", drive.localizer.getPose().heading.toDouble());
//            telemetry.addData("otos:heading", otos.getPose().heading.toDouble());
            telemetry.update();
            waitForStart();
//            while (opModeIsActive()){
//                drive.updatePoseEstimate();

//                telemetry.update();
//            }
            NonDrivingActions actions = new NonDrivingActions();

            Pose2d target = new Pose2d(2, 0, Math.toRadians(-40));
            double delta_x = target.position.x - beginPose.position.x;
            double delta_y = target.position.y - beginPose.position.y;
            double delta_a = target.heading.toDouble() - beginPose.heading.toDouble();
            double tangent = Math.atan2(delta_y, delta_x);
            drive.extraCorrection = true;

            Actions.runBlocking(
                    new ParallelAction(
                            drive.actionBuilder(beginPose)
                                    .setTangent(tangent)
//                                    .lineToXLinearHeading(target.position.x,)
                                    .splineToLinearHeading(
                                            target,
                                            tangent,
                                            new AngularVelConstraint(Math.toRadians(20))
                                    )

//                                    .turnTo(Math.PI*-.75, new TurnConstraints(2, -2, 2))
//                                    .waitSeconds(3)
//                                    .turnTo(0, new TurnConstraints(2, -2, 2))
                                    //                        .splineToConstantHeading(new Vector2d(30, 30), Math.PI / 2)
                                    //                        .splineToConstantHeading(new Vector2d(0, 60), Math.PI)
                                    //
                                    //                        .setTangent(0)
                                    //                        .splineToConstantHeading(new Vector2d(30, 30), -Math.PI / 2)
                                    //                        .splineToConstantHeading(new Vector2d(0, 0), Math.PI)
                                    .build(),
                            actions.getPos()
                    ));
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }

    class NonDrivingActions {
        public Action getPos() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    telemetry.addData("Heading: ", drive.localizer.getPose().heading.toDouble());
                    telemetry.update();
                    return opModeIsActive();
                }
            };
        }
    }
}
