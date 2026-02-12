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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "AutoBlueStateCorner", group = "A")
public final class AutoBlueStateCorner extends LinearOpMode {

    MecanumDrive drive;
    private ElapsedTime shootTime = new ElapsedTime();
    private ElapsedTime startShootTime = new ElapsedTime();
    private ElapsedTime pickUpTimer = new ElapsedTime();
    private ElapsedTime startTime = new ElapsedTime();
    private ElapsedTime camTimer = new ElapsedTime();

    int ballNumber = 0;
    private DcMotor shoot_l;
    private DcMotor arm;
    private DcMotor intake;
    private CRServo magazen;
    private DcMotor back_l;
    private DcMotor front_r;
    private DcMotor back_r;
    private DcMotor front_l;
    private DcMotor shoot_top;
    private DcMotor shoot_main;
    private Servo lever;
    int magTargetPos = 0;
    boolean traySpun = false;
    String pattern = "XXX";
    TrayController trayController;
    PIDController shootController;
    PIDController shootLineUpController;
    PipelineLibrary camLibrary;
    int topShootTarget = 0;
    int mainShootTarget = 0;

    boolean updateTrayCam = false;
    PIDController topShootController;
    PIDController mainShootController;
    double topShootPower = 0;
    double mainShootPower = 0;
    boolean shooting = false;
    @Override
    public void runOpMode() throws InterruptedException {
        camTimer.reset();

        camLibrary = new PipelineLibrary(hardwareMap);

        trayController = new TrayController(hardwareMap);
        trayController.setAutoSortTo(false);
        trayController.resetTrayPosition();
        trayController.setZoneColors(new int[]{1, 1, 2, 0, 0});

        topShootController = new PIDController(0.0009, 0.00008, 0.0000);
        mainShootController = new PIDController(0.006, 0.0000, 0.00008);


        shootLineUpController = new PIDController(1.2,0,0.05);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotor.class, "intake");
        magazen = hardwareMap.get(CRServo.class, "magazen");
        back_l = hardwareMap.get(DcMotor.class, "back_l");
        front_r = hardwareMap.get(DcMotor.class, "front_r");
        back_r = hardwareMap.get(DcMotor.class, "back_r");
        front_l = hardwareMap.get(DcMotor.class, "front_l");

        shoot_top = hardwareMap.get(DcMotor.class, "shoot_l");
        shoot_main = hardwareMap.get(DcMotor.class, "shoot_main");
        lever = hardwareMap.get(Servo.class, "lever");

        shoot_top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_main.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot_main.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_main.setDirection(DcMotor.Direction.REVERSE);


//        Pose2d initialPose = new Pose2d(50, 52, -Math.PI/180 * 36);
        Pose2d initialPose = new Pose2d(-63, 14, Math.PI*.5);


        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(initialPose)
                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(-55, 14, Math.PI*.66), 0);
                .splineToConstantHeading(new Vector2d(-53, 16), 0)
                .turnTo(Math.PI*.66);
        TrajectoryActionBuilder driveToPickup = drive.actionBuilder(new Pose2d(-53, 16, Math.PI*.66))
                .turnTo(Math.PI*.60)
                .setTangent(Math.PI*.5)
                .splineToConstantHeading(new Vector2d(-58, 64), Math.PI*.5)
                .setTangent(Math.PI*-.5)
                .splineToSplineHeading(new Pose2d(-60,58, Math.PI*.55),-Math.PI*.6)
                .setTangent(Math.PI*.6)
                .splineToConstantHeading(new Vector2d(-62, 64), Math.PI*.5)
                .setTangent(Math.PI*-.4)
                .splineToConstantHeading(new Vector2d(-53, 16), Math.PI*-.5)
                .turnTo(Math.PI*.66);
        TrajectoryActionBuilder driveToPickup2 = drive.actionBuilder(new Pose2d(-53, 16, Math.PI*.66))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-41,12,Math.PI*.5),0)
                .splineToConstantHeading(new Vector2d(-37, 55), Math.PI*.5)
                .setTangent(Math.PI*-.6)
                .splineToSplineHeading(new Pose2d(-53, 16, Math.PI*.62), Math.PI*-.5);
        TrajectoryActionBuilder driveToPickup3 = drive.actionBuilder(new Pose2d(-53, 16, Math.PI*.62))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-16,12,Math.PI*.5),0)
                .splineToConstantHeading(new Vector2d(-12, 55), Math.PI*.5)
                .setTangent(Math.PI*-.75)
                .splineToSplineHeading(new Pose2d(-55, 16, Math.PI*.64), Math.PI*-.5);
//                .setTangent(Math.PI*.0)
//                .splineToSplineHeading(new Pose2d(45, 40, Math.PI*.25), Math.PI*0);

//        TrajectoryActionBuilder holdShootPos = drive.actionBuilder(new Pose2d(12, 12, Math.PI*-.75000001))
//                .turnTo(Math.PI*-.75);

        TrajectoryActionBuilder driveToPark = drive.actionBuilder(new Pose2d(-55, 16, Math.PI*.64))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-36, 20), Math.PI*.75);

        Action driveToShootAction = driveToShoot.build();
        Action driveToPickupAction = driveToPickup.build();
        Action driveToPickup2Action = driveToPickup2.build();
        Action driveToPickup3Action = driveToPickup3.build();
//        Action holdShootPosAction = holdShootPos.build();
        Action driveToParkAction = driveToPark.build();

//        while (opModeInInit()) {
//            pass;
//        }

//        telemetry.addData("ready to start", "true");
//        telemetry.update();

        while (opModeInInit()){
            List<AprilTagDetection> currentDetections = camLibrary.aprilTagDetectionPipeline.getLatestDetections();

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
//                }
            }   // end for() loop
//            telemetry.addData("target", 0);
//            telemetry.addData("current", ((DcMotorEx) shoot_l).getVelocity());
//            telemetry.addData("trayTarget", trayController.magPos);
//            telemetry.addData("trayCurrent", intake.getCurrentPosition());
            telemetry.addData("ball code", pattern);
            telemetry.update();
        }

        waitForStart();
        startTime.reset();

        if (Objects.equals(pattern, "XXX")){
            pattern = "PPG";
        }
        PoseStorage.ballPattern = pattern;
        PoseStorage.alianceIsBlue = true;
        trayController.setCode(pattern);
//        MagazenControler magControler = new MagazenControler();
        magTargetPos = 0;

        NonDrivingActions actions = new NonDrivingActions();

        Actions.runBlocking(
                new ParallelAction(
                        actions.updateTray(),
                        new SequentialAction(
                                new ParallelAction(
                                        actions.startShooter(),
                                        driveToShootAction
                                ),
                                actions.Shoot(),
                                actions.turnOnIntake(),
                                driveToPickupAction,
                                actions.updateBallPositions(),
                                actions.startShooter(),
                                actions.Shoot(),
                                driveToPickup2Action,
                                actions.updateBallPositions(),
                                actions.startShooter(),
                                actions.Shoot(),
                                actions.turnOffIntake(),
                                driveToParkAction

                                //                        actions.turnOnIntake(),
                                //                        driveToPickup2Action,
                                //                        actions.turnOffIntake(),
                                //                        actions.Shoot(1),
                                //                        driveToParkAction
                        )
                )
        );

        PoseStorage.currentPose = drive.localizer.getPose();
    }

//    private void initAprilTag() {
//
//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//
//                // The following default settings are available to un-comment and edit as needed.
//                //.setDrawAxes(false)
//                //.setDrawCubeProjection(false)
//                //.setDrawTagOutline(true)
//                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//
//                // == CAMERA CALIBRATION ==
//                // If you do not manually specify calibration parameters, the SDK will attempt
//                // to load a predefined calibration for your camera.
//                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                // ... these parameters are fx, fy, cx, cy.
//
//                .build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        //aprilTag.setDecimation(3);
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(aprilTag);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Disable or re-enable the aprilTag processor at any time.
//        //visionPortal.setProcessorEnabled(aprilTag, true);
//
//    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
//    private void telemetryAprilTag() {
//
//
//
//    }   // end method telemetryAprilTag()
    class NonDrivingActions {
        int[] ballColors = {0,0,0,0,0};
        public Action updateTray() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    PoseStorage.currentPose = drive.localizer.getPose();
                    ballColors = camLibrary.trayPipeline.getBallColors();

                    if (ballColors[0] == 0 || ballColors[1] == 0 || ballColors[2] == 0) {
                        pickUpTimer.reset();
                    }

                    if (intake.getPower() != 0){
                        if (pickUpTimer.milliseconds() > 1000) {
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

                    if (mainShootTarget != 0){
                        topShootPower = topShootTarget*0.00042+topShootController.update(topShootTarget, ((DcMotorEx) shoot_top).getVelocity());
                        mainShootPower = mainShootTarget*0.0001+mainShootController.update(mainShootTarget, ((DcMotorEx) shoot_main).getVelocity());
                    } else {
                        topShootPower = 0;
                        mainShootPower = 0;
                    }

                    shoot_top.setPower(topShootPower);
                    shoot_main.setPower(mainShootPower);

                    if (startShootTime.milliseconds() > 250 && mainShootTarget != 0) {
                        lever.setPosition(1);
                    } else {
                        lever.setPosition(.8);
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
                    int[] ballPositions = ballColors;
//                    boolean greenBall = false;
//                    if (ballPositions[0] == 2 || ballPositions[1] == 2 || ballPositions[2] == 2) {
//                        greenBall = true;
//                    }
//                    for (int i = 0; i <= 2; i++) {
//                        if (ballPositions[i] == 0){
//                            if (greenBall) {
//                                ballPositions[i] = 1;
//                            } else {
//                                ballPositions[i] = 2;
//                            }
//                        }
//                    }


                    trayController.setZoneColors(ballPositions);
                    return false;
                }
            };
        }
        public Action startShooter() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (startTime.milliseconds() > 1000){
                        topShootController.resetTimer();
                        mainShootController.resetTimer();
                        updateTrayCam = false;
                        shooting = false;
                        trayController.setAutoSortTo(false);
                        startShootTime.reset();
                        trayController.spinToShootReady();
                        topShootTarget = 1400;
                        mainShootTarget = 1170;
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }

        double convertApriltagPosToFlatDistance(AprilTagPose pose) {
            return Math.sqrt(Math.pow(     Math.sqrt(Math.pow(pose.x*39.37,2)+Math.pow(pose.y*39.37,2)+Math.pow(pose.z*39.37,2))    ,2)-Math.pow(16.25,2));
        }

        int spinCount = 0;
        public Action Shoot() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!shooting && startShootTime.milliseconds() > 700) {
                        shootTime.reset();
                        spinCount = 0;
                        shooting = true;
                    }
                    if (shootTime.milliseconds() > spinCount * 650 && shooting) {
                        trayController.spinLeft_half();
                        spinCount ++;
                    }

                    if (camLibrary.aprilTagDetectionPipeline.getDetectionsUpdate() != null) {
                        camTimer.reset();
                    }
                    double turnError = 0;
                    if (camTimer.milliseconds() <= 250){
                        for (AprilTagDetection detection : camLibrary.aprilTagDetectionPipeline.getLatestDetections()) {
                            if (detection.id == 20 || detection.id == 24) {
                                double distanceToTag = convertApriltagPosToFlatDistance(detection.pose);
                                double angleToTag = Math.atan2(detection.pose.x*39.37,distanceToTag);
                            turnError = -angleToTag+Math.toRadians(3);
                            }
                        }
                    }

                    telemetry.addData("turnError", turnError);
                    telemetry.update();

                    double turnPower = shootLineUpController.update(turnError, 0);
                    front_l.setPower(turnPower * -.7);
                    back_l.setPower(turnPower * -.7);
                    front_r.setPower(-turnPower * -.7);
                    back_r.setPower(-turnPower * -.7);
//
                    if (shooting && spinCount >= 6) {
                        front_l.setPower(0);
                        back_l.setPower(0);
                        front_r.setPower(0);
                        back_r.setPower(0);
                        drive.extraCorrection = false;
                        topShootTarget = 0;
                        mainShootTarget = 0;
                        shooting = false;
//                        trayController.setCode(pattern);
                        updateTrayCam = true;
                        trayController.setAutoSortTo(true);
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }
//        public Action GetTagId() {
//            return new Action() {
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//
//                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//                    // Step through the list of detections and display info for each one.
//                    for (AprilTagDetection detection : currentDetections) {
//                            if (detection.ftcPose.yaw > 0) {
//                                int tagId = detection.id;
//                                if (tagId == 21){
//                                    trayController.setCode("GPP");
//                                    pattern = "GPP";
////                                    ballPattern = new int[]{1, 2, 3};
//                                } else if (tagId == 22){
//                                    trayController.setCode("PGP");
//                                    pattern = "PGP";
////                                    ballPattern = new int[]{3, 1, 2};
//                                } else {
////                                    ballPattern = new int[]{3, 2, 1};
//                                    trayController.setCode("PPG");
//                                    pattern = "PPG";
//                                }
//                            }
//                    }   // end for() loop
//                    telemetry.addData("x",drive.localizer.getPose().position.x);
//                    telemetry.addData("y",drive.localizer.getPose().position.y);
//                    telemetry.addData("ball code", pattern);
////                    telemetry.addData("test",ballPattern[0]+", "+ballPattern[1]+", "+ballPattern[2]);
//                    telemetry.update();
//                    if (true) {
//                        visionPortal.close();
//                        return false;
//                    } else {
//                        return true;
//                    }
//                }
//            };
//        }
    }
}

