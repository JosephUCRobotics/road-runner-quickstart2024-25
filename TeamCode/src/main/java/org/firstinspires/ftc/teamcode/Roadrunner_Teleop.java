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

@Disabled

@TeleOp(name="Roadrunner_Teleop")

public class Roadrunner_Teleop extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    Action drive_action = null;

    MecanumDrive drive;
    private DcMotor back_l;
    private DcMotor front_r;
    private DcMotor back_r;
    private DcMotor front_l;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, 0);

        drive = new MecanumDrive(hardwareMap, initialPose);

        double driverOrientedHeading = 0;



        back_l = hardwareMap.get(DcMotor.class, "back_l");
        front_r = hardwareMap.get(DcMotor.class, "front_r");
        back_r = hardwareMap.get(DcMotor.class, "back_r");
        front_l = hardwareMap.get(DcMotor.class, "front_l");
//
//        // Put initialization blocks here.
//        front_l.setDirection(DcMotor.Direction.FORWARD);
//        back_l.setDirection(DcMotor.Direction.REVERSE);
//        front_r.setDirection(DcMotor.Direction.REVERSE);
//        back_r.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                TelemetryPacket packet = new TelemetryPacket();


                if (gamepad1.aWasPressed()){




                    Pose2d current = new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble());
                    Pose2d target = new Pose2d(10, 0, 0);
                    double delta_x = target.position.x - current.position.x;
                    double delta_y = target.position.y - current.position.y;
                    double delta_a = target.heading.toDouble() - current.heading.toDouble();
                    if (delta_x!=0||delta_y!=0|delta_a!=0){
                        double tangent = Math.atan2(delta_y, delta_x);

                        TrajectoryActionBuilder drive_Path = drive.actionBuilder(current)
                                .setTangent(tangent)
                                .splineToLinearHeading(target, tangent, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50));
                        drive_action = drive_Path.build();
//                        runningActions.add(new SequentialAction(
//                                drive_action
//                        ));
                    }
                }

                if (gamepad1.a){
//                    List<Action> newActions = new ArrayList<>();
//                    for (Action action : runningActions) {
//                        action.preview(packet.fieldOverlay());
//                        if (action.run(packet)){
//                            newActions.add(action);
//                        }
//                    }
//
//                    runningActions = newActions;

                    if(drive_action != null){
                        drive_action.preview(packet.fieldOverlay());
                        if(!drive_action.run(packet)){
                            drive_action = null;
                        }
                    }
                } else {
                    drive_action = null;


                    double heading = drive.localizer.getPose().heading.toDouble();
                    if (gamepad1.start){
                        driverOrientedHeading = heading;
                    }
                    heading-= driverOrientedHeading;
                    double sticky = gamepad1.left_stick_y * Math.cos(heading) + gamepad1.left_stick_x * Math.sin(heading);
                    double stickx = gamepad1.left_stick_x * Math.cos(heading) - gamepad1.left_stick_y * Math.sin(heading);

                    front_l.setPower(((sticky - stickx) - (gamepad1.right_trigger - gamepad1.left_trigger))*-1);
                    back_l.setPower(((sticky + stickx) - (gamepad1.right_trigger - gamepad1.left_trigger))*-1);
                    front_r.setPower((sticky + stickx + (gamepad1.right_trigger - gamepad1.left_trigger))*-1);
                    back_r.setPower(((sticky - stickx) + (gamepad1.right_trigger - gamepad1.left_trigger))*-1);

                    drive.updatePoseEstimate();
                }

                dash.sendTelemetryPacket(packet);
            }
        }
    }
}
