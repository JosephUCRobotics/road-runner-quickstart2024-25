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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.List;
import java.util.Objects;
import java.util.regex.Pattern;

@Autonomous(name = "AutoBlue", group = "A")
public final class AutoBlue extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
//    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
//    private VisionPortal visionPortal;

    MecanumDrive drive;
    private ElapsedTime shootTime = new ElapsedTime();

    int ballNumber = 0;
//    int[] ballPattern = {1,2,3};

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
    int[] magPosList  = {0, 2731, 2731*2, 2731*3};
    TrayController trayController;
    PIDController shootController;
    PIDController shootLineUpController;
    TwoCamLibrary camLibrary;

    boolean updateTrayCam = true;
    @Override
    public void runOpMode() throws InterruptedException {

//        initAprilTag();
        camLibrary = new TwoCamLibrary(hardwareMap);

        trayController = new TrayController(hardwareMap);
        trayController.setAutoSortTo(true);
        trayController.resetTrayPosition();
        trayController.setZoneColors(new int[]{1, 2, 1, 0, 0});

        shootController = new PIDController(0.006, 0, 0);

        shootLineUpController = new PIDController(1.2,0,0.05);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        ElapsedTime myElapsedTime;
        double Drive_Speed;



        boolean left_bumper_down = false;
        boolean right_bumper_down = false;

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
        Pose2d initialPose = new Pose2d(-65, 14, Math.PI);


        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, 12, Math.PI*-.75), 0);
        TrajectoryActionBuilder driveToPickup = drive.actionBuilder(new Pose2d(12, 12, Math.PI*-.75))
                .turnTo(Math.PI*.5)
                .setTangent(Math.PI*.5)
                .splineToConstantHeading(new Vector2d(12, 44), Math.PI*.5)
                .setTangent(Math.PI*-.5)
                .splineToConstantHeading(new Vector2d(12, 12), Math.PI*-.5)
                .turnTo(Math.PI*-.75);
        TrajectoryActionBuilder driveToPickup2 = drive.actionBuilder(new Pose2d(12, 12, Math.PI*-.75))
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(-8,12,Math.PI*.5),Math.PI)
                .splineToConstantHeading(new Vector2d(-12, 44), Math.PI*.5)
                .setTangent(Math.PI*-.5)
                .splineToSplineHeading(new Pose2d(12, 12, Math.PI*-.75), Math.PI*-.5);
//                .setTangent(Math.PI*.0)
//                .splineToSplineHeading(new Pose2d(45, 40, Math.PI*.25), Math.PI*0);

//        TrajectoryActionBuilder holdShootPos = drive.actionBuilder(new Pose2d(12, 12, Math.PI*-.75000001))
//                .turnTo(Math.PI*-.75);

        TrajectoryActionBuilder driveToPark = drive.actionBuilder(new Pose2d(12, 12, Math.PI*-.75))
                .setTangent(Math.PI*.75)
                .splineToConstantHeading(new Vector2d(0, 24), Math.PI*.75);

        Action driveToShootAction = driveToShoot.build();
        Action driveToPickupAction = driveToPickup.build();
        Action driveToPickup2Action = driveToPickup2.build();
//        Action holdShootPosAction = holdShootPos.build();
        Action driveToParkAction = driveToPark.build();

//        while (opModeInInit()) {
//            pass;
//        }

//        telemetry.addData("ready to start", "true");
//        telemetry.update();

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
//                }
            }   // end for() loop
            telemetry.addData("target", 0);
            telemetry.addData("current", ((DcMotorEx) shoot_l).getVelocity());
            telemetry.addData("trayTarget", trayController.magPos);
            telemetry.addData("trayCurrent", intake.getCurrentPosition());
            telemetry.addData("ball code", pattern);
            telemetry.update();
        }

        waitForStart();
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
//                                actions.GetTagId(),
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
                    telemetry.addData("target", velocity);
                    telemetry.addData("current", ((DcMotorEx) shoot_l).getVelocity());
                    telemetry.addData("trayTarget", trayController.magPos);
                    telemetry.addData("trayCurrent", intake.getCurrentPosition());
                    telemetry.update();
                    int spinTimeChange = 0;
//                    if (ballNumber > 1) {
//                        if (Math.abs(magPos-ballPattern[ballNumber-2])==1){
//                            spinTimeChange = 200;
//                        }
//                    }
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

