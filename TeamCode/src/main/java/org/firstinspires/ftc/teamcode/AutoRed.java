package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import java.util.Objects;

@Autonomous(name = "AutoRed", group = "A")
public final class AutoRed extends LinearOpMode {

    MecanumDrive drive;
    private ElapsedTime shootTime = new ElapsedTime();

    int ballNumber = 0;

    private DcMotor shoot_l;
    private DcMotor arm;
    private DcMotor intake;
    private CRServo magazen;
    private DcMotor back_l;
    private DcMotor front_r;
    private DcMotor back_r;
    private DcMotor front_l;
    int magTargetPos = 0;
    boolean traySpun = false;
    String pattern = "XXX";
    TrayController trayController;
    PIDController shootController;
    PIDController shootLineUpController;
    TwoCamLibrary camLibrary;

    boolean updateTrayCam = true;
    @Override
    public void runOpMode() throws InterruptedException {

        camLibrary = new TwoCamLibrary(hardwareMap);

        trayController = new TrayController(hardwareMap);
        trayController.setAutoSortTo(true);
        trayController.resetTrayPosition();
        trayController.setZoneColors(new int[]{1, 2, 1, 0, 0});

        shootController = new PIDController(0.006, 0, 0);

        shootLineUpController = new PIDController(1.2,0,0.05);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        shoot_l = hardwareMap.get(DcMotor.class, "shoot_l");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        magazen = hardwareMap.get(CRServo.class, "magazen");
        back_l = hardwareMap.get(DcMotor.class, "back_l");
        front_r = hardwareMap.get(DcMotor.class, "front_r");
        back_r = hardwareMap.get(DcMotor.class, "back_r");
        front_l = hardwareMap.get(DcMotor.class, "front_l");

        // Put initialization blocks here.
        shoot_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);

//        Pose2d initialPose = new Pose2d(50, 52, -Math.PI/180 * 36);
        Pose2d initialPose = new Pose2d(-65, -14, Math.PI);


        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, -12, Math.PI*.75), 0);
        TrajectoryActionBuilder driveToPickup = drive.actionBuilder(new Pose2d(12, -12, Math.PI*.75))
                .turnTo(Math.PI*-.5)
                .setTangent(Math.PI*-.5)
                .splineToConstantHeading(new Vector2d(12, -44), Math.PI*-.5)
                .setTangent(Math.PI*.5)
                .splineToConstantHeading(new Vector2d(12, -12), Math.PI*.5)
                .turnTo(Math.PI*.75);
        TrajectoryActionBuilder driveToPickup2 = drive.actionBuilder(new Pose2d(12, -12, Math.PI*.75))
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(-6,-12,Math.PI*-.5),Math.PI)
                .splineToConstantHeading(new Vector2d(-12, -44), Math.PI*-.5)
                .setTangent(Math.PI*.5)
                .splineToSplineHeading(new Pose2d(12, -12, Math.PI*.75), Math.PI*.5);

        TrajectoryActionBuilder driveToPark = drive.actionBuilder(new Pose2d(12, -12, Math.PI*.75))
                .setTangent(Math.PI*-.75)
                .splineToConstantHeading(new Vector2d(0, -24), Math.PI*-.75);

        Action driveToShootAction = driveToShoot.build();
        Action driveToPickupAction = driveToPickup.build();
        Action driveToPickup2Action = driveToPickup2.build();
        Action driveToParkAction = driveToPark.build();

        while (opModeInInit()){
            List<AprilTagDetection> currentDetections = camLibrary.aprilTagProcessor.getDetections();

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
//                if (detection.ftcPose.yaw > 0) {
                int tagId = detection.id;
                if (tagId != 24 && tagId != 20) {
                    if (tagId == 21){
                        pattern = "GPP";
                    } else if (tagId == 22){
                        pattern = "PGP";
                    } else {
                        pattern = "PPG";
                    }
                }
            }
            telemetry.addData("ball code", pattern);
            telemetry.update();
        }

        waitForStart();
        if (Objects.equals(pattern, "XXX")){
            pattern = "PPG";
        }
        PoseStorage.ballPattern = pattern;
        PoseStorage.alianceIsBlue = false;
        trayController.setCode(pattern);
        magTargetPos = 0;

        NonDrivingActions actions = new NonDrivingActions();

        Actions.runBlocking(
                new ParallelAction(
                        actions.updateTray(),
                        new SequentialAction(
                                actions.startShooter(),
                                driveToShootAction,
                                actions.Shoot(3),
                                actions.turnOnIntake(),
                                actions.startShooter(),
                                driveToPickupAction,
                                actions.updateBallPositions(),
                                actions.Shoot(3),
                                actions.startShooter(),
                                driveToPickup2Action,
                                actions.updateBallPositions(),
                                actions.Shoot(3),
                                actions.turnOffIntake(),
                                driveToParkAction
                        )
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }

    class NonDrivingActions {
        public Action updateTray() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    PoseStorage.currentPose = drive.localizer.getPose();
                    int[] ballColors = camLibrary.trayProcessor.getBallColors();
                    if (intake.getPower() != 0){
                        if (ballColors[0] != 0 && ballColors[1] != 0 && ballColors[2] != 0) {
                            intake.setPower(-1);
                        } else {
                            intake.setPower(1);
                        }
                    }
                    if (updateTrayCam) {
                        trayController.update(ballColors);
                    } else {
                        trayController.update();
                    }

                    return opModeIsActive();
                }
            };
        }
        public Action turnOnIntake() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intake.setPower(1);
                    trayController.setZoneColors(new int[]{1, 1, 2, 0, 0});
                    return false;
                }
            };
        }

        public Action turnOffIntake() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intake.setPower(0);
                    return false;
                }
            };
        }

        public Action updateBallPositions() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    int[] ballPositions = camLibrary.trayProcessor.getBallColors();

                    trayController.setZoneColors(ballPositions);
                    return false;
                }
            };
        }
        public Action startShooter() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    double velocity = 7 * 55 + 1265;

                    ((DcMotorEx) shoot_l).setPower(velocity/2650);

                    return false;
                }
            };
        }

        public Action Shoot(int endingPos) {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (ballNumber == 0) {
                        shootLineUpController.resetTimer();
                        drive.extraCorrection = true;
                        shootTime.reset();
                        shootController.resetTimer();
                        ballNumber = 1;
                        updateTrayCam = false;
                        trayController.setAutoSortTo(false);
                    }
                    double velocity = 7 * 55 + 1265;

                    ((DcMotorEx) shoot_l).setPower(velocity/2650+shootController.update(velocity, ((DcMotorEx) shoot_l).getVelocity()));
                    int spinTimeChange = 0;

                    if (shootTime.milliseconds() < 750-spinTimeChange) {
                        if (!traySpun){
                            trayController.spinToNextInCode();
                        }
                        traySpun = true;
                    } else if (shootTime.milliseconds() < 1100-spinTimeChange) {
                        arm.setPower(1);
                    } else if (shootTime.milliseconds() < 1400-spinTimeChange) {
                        arm.setPower(0.0);
                    } else {
                        arm.setPower(0);
                        shootTime.reset();
                        ballNumber += 1;
                        traySpun = false;
                    }
                    double turnError = 0;
                    for (AprilTagDetection detection : camLibrary.aprilTagProcessor.getDetections()) {
                        if (detection.id == 20 || detection.id == 24) {
                            turnError = Math.toRadians(-detection.ftcPose.bearing);
                        }
                    }
                    double turnPower = shootLineUpController.update(turnError, 0);
                    front_l.setPower(turnPower * -.7);
                    back_l.setPower(turnPower * -.7);
                    front_r.setPower(-turnPower * -.7);
                    back_r.setPower(-turnPower * -.7);

                    if (ballNumber > 3) {
                        front_l.setPower(0);
                        back_l.setPower(0);
                        front_r.setPower(0);
                        back_r.setPower(0);
                        drive.extraCorrection = false;
                        ((DcMotorEx) shoot_l).setPower(0);
                        ballNumber = 0;
                        trayController.setCode(pattern);
                        updateTrayCam = true;
                        trayController.setAutoSortTo(true);
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }
    }
}

