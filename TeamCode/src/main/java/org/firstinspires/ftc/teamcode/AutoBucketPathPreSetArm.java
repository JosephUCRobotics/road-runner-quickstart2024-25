package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// adb connect 192.168.43.1:5555
@Autonomous(name = "CameraDriveTest", group = "A")
public final class AutoBucketPathPreSetArm extends LinearOpMode {
    MecanumDrive drive;
    private ElapsedTime camTime = new ElapsedTime();
    private ElapsedTime pickupTime = new ElapsedTime();
    private DcMotorEx shoulder;
    private Servo clawUp;
    private CRServo clawWheels;

    @Override
    public void runOpMode() throws InterruptedException {
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulder.setDirection(DcMotorEx.Direction.FORWARD);
        shoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        clawUp = hardwareMap.get(Servo.class, "clawUp");
        clawWheels = hardwareMap.get(CRServo.class, "clawWheels");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraVariables.hardwareMap = hardwareMap;
        MainVision vision = new MainVision();

//        Pose2d initialPose = new Pose2d(0, 0, -Math.PI*.75);
        Pose2d initialPose = new Pose2d(24, 0, Math.PI*.75);


        drive = new MecanumDrive(hardwareMap, initialPose);

//        TrajectoryActionBuilder firstPathToBuckets = drive.actionBuilder(initialPose)
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(24, 0, -Math.PI*.75), 0, null, new ProfileAccelConstraint(-10, 10))
//                .turnTo(Math.PI*.75);

        TrajectoryActionBuilder secondTrajectoryAction = drive.actionBuilder(new Pose2d(24, 0, Math.PI*.75))
                .setTangent(-Math.PI*.5)
                .splineToSplineHeading(new Pose2d(24, -10, Math.PI*.75), -Math.PI*.5, new TranslationalVelConstraint(15), new ProfileAccelConstraint(-10, 10));



//        Action firstTrajectoryActionChosen = firstPathToBuckets.build();
        Action secondTrajectoryActionChosen = secondTrajectoryAction.build();
        CameraTarget redBlock = new CameraTarget();

        while (opModeInInit()) {
            vision.setManualExposure(CameraVariables.exposure, CameraVariables.gain);
            redBlock.setPositionWithRobotPos(drive.localizer.getPose());
//            double angleRight = -CameraVariables.distanceX1 / 14;
//            double angleUp = CameraVariables.distanceY1 / 14-CameraVariables.camOffset;
//            double c = CameraVariables.camHeight;
//            double x = Math.tan(Math.toRadians(90 + angleUp)) * c;
//            double y = Math.tan(Math.toRadians(angleRight)) * x;
//            double C = 180 - angle1 - angle2;
//            double a = Math.sin(Math.toRadians(angle1)) * (c/Math.sin(Math.toRadians(C)));
//
//            double z = (Math.sin(Math.toRadians(angle2))*a)*.8-17;
//
//            double angle = angle2-angle1;
            telemetry.addData("robotHeading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("robotHeading", drive.localizer.getPose().heading.toDouble());
            telemetry.addData("robotX: ", drive.localizer.getPose().position.x);
            telemetry.addData("robotY: ", drive.localizer.getPose().position.y);
            telemetry.addData("targetX: ", redBlock.getFieldXPos());
            telemetry.addData("targetY: ", redBlock.getFieldYPos());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            redBlock.setPositionWithRobotPos(drive.localizer.getPose());

            telemetry.addData("robotHeading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("robotX: ", drive.localizer.getPose().position.x);
            telemetry.addData("robotY: ", drive.localizer.getPose().position.y);
            telemetry.addData("targetX: ", redBlock.getFieldXPos());
            telemetry.addData("targetY: ", redBlock.getFieldYPos());
            telemetry.update();
            drive.updatePoseEstimate();
        }
//        CameraActions ca = new CameraActions();
//
//        Actions.runBlocking(
//
//                    new ParallelAction(
//                            ca.updatePositionWithCameras(),
//                            new SequentialAction(
//                                    secondTrajectoryActionChosen,
//                                    ca.startClaw(),
//                                    ca.moveArmDown(),
//                                    ca.moveArmUp(),
//                                    ca.stopClaw()
//                                    )
//                    )
//
//        );
    }
    class CameraActions {
        public Action updatePositionWithCameras() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    double target = 15;
                    double angleRight = -CameraVariables.distanceX1 / 14;
                    double angleUp = CameraVariables.distanceY1 / 14-CameraVariables.camOffset;
                    double c = CameraVariables.camHeight;
                    double x = Math.tan(Math.toRadians(90 + angleUp)) * c;
                    double y = Math.tan(Math.toRadians(angleRight)) * x;
                    double xerror = x-target;
                    double yerror = y+1;
                    if (Math.abs(xerror) < 3){
                        camTime.reset();
                    }
                    // todo
//                    if (camTime.seconds() > -1){
//                        drive.setcamErr(true);
//                    }

//                    drive.setPose(new Pose2d(24, z-32, Math.PI*.75));

                    telemetry.addData("angleUp: ", angleUp);
                    telemetry.addData("angleRight: ", angleRight);
                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.update();

                    drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x + ((24+yerror)-drive.localizer.getPose().position.x)*.04, drive.localizer.getPose().position.y +((xerror-10)-drive.localizer.getPose().position.y)*.03, drive.localizer.getPose().heading.toDouble()));


                    return !gamepad1.a;
                }
            };
        }
        public Action moveArmDown() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {

                    double power = (0 - shoulder.getCurrentPosition()) * -0.05;
                    if (power > 0.3) {
                        power = 0.3;
                    } else if (power < -0.3) {
                        power = -0.3;
                    }
                    shoulder.setPower(power);
                    if (Math.abs(0 - shoulder.getCurrentPosition()) > 100) {
                        pickupTime.reset();
                    }
                    if (pickupTime.seconds()>1){
                        shoulder.setPower(0);
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }
        public Action moveArmUp() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    double power = (-500 - shoulder.getCurrentPosition()) * -0.05;
                    if (power > 0.5) {
                        power = 0.5;
                    } else if (power < -0.3) {
                        power = -0.3;
                    }
                    shoulder.setPower(power);
                    if (Math.abs(-500 - shoulder.getCurrentPosition()) < 100) {
                        shoulder.setPower(0);
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }
        public Action startClaw() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    clawUp.setPosition(0.35);
                    clawWheels.setPower(-1);
                    return false;

                }
            };
        }
        public Action stopClaw() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    clawWheels.setPower(0);
                    return false;

                }
            };
        }
    }
}

