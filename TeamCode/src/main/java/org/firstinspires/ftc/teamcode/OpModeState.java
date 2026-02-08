package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp(name="Driver State")
public class OpModeState extends LinearOpMode {
    MecanumDrive drive;
    private DcMotor shoot_top;
    private DcMotor shoot_main;
    private DcMotor back_l;
    private DcMotor front_r;
    private DcMotor back_r;
    private DcMotor intake;
    private DcMotor front_l;
    private Servo lever;

    private List<Action> runningActions = new ArrayList<>();
    private double turnError = 0;
    private double turnPower = 0;
    private double xPower = 0;
    private double yPower = 0;
    Vector2d tagPose = new Vector2d(59, 56);
    Vector2d goalPos = new Vector2d(69, 69);
    AprilTagDetection detectedTag = null;
    private ElapsedTime shootTime = new ElapsedTime();
    private ElapsedTime pickUpTimer = new ElapsedTime();
    private ElapsedTime aprilTagTimer = new ElapsedTime();

    public static double shooterTuner_topShootVelocity = 1350;
    public static double shooterTuner_mainShootVelocity = 1350;
    public static double shooterTuner_topP = 0.001;
    public static double shooterTuner_topI = 0.00001;
    public static double shooterTuner_topD = 0.00001;
    public static double shooterTuner_topF = 0.00042;
    public static double shooterTuner_mainP = 0.006;
    public static double shooterTuner_mainI = 0.00005;
    public static double shooterTuner_mainD = 0.00005;
    public static double shooterTuner_mainF = 0.00042;
    public  static double drivep = 0.03;
    public  static double drivei = 0;
    public  static double drived = 0.001;
    public  static double drivef = 0.05;
    public  static double drivemax = 0.8;
    public  static double turnMax = 0.8;
    public  static double trayMaxPow = .1;

    double barringOffsetOverride = 0;
    double topShootPowOverride = 0;
    double mainShootPowOverride = 0;

    boolean cameraCrashed = false;

    @Override
    public void runOpMode() throws IllegalStateException{
        Pose2d initialPose = PoseStorage.currentPose;

        drive = new MecanumDrive(hardwareMap, initialPose);
        TrayController trayController = new TrayController(hardwareMap);
        trayController.setAutoSortTo(true);
        trayController.setAutoSortTo(false);
        TwoCamLibrary camLibrary = new TwoCamLibrary(hardwareMap);
        ElapsedTime myElapsedTime;
        double Drive_Speed = .7;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (!PoseStorage.alianceIsBlue){
            tagPose = new Vector2d(59, -56);
            goalPos = new Vector2d(69, -69);
        }


        Action drive_action = null;

        back_l = hardwareMap.get(DcMotor.class, "back_l");
        front_r = hardwareMap.get(DcMotor.class, "front_r");
        back_r = hardwareMap.get(DcMotor.class, "back_r");
        intake = hardwareMap.get(DcMotor.class, "intake");
        front_l = hardwareMap.get(DcMotor.class, "front_l");


        shoot_top = hardwareMap.get(DcMotor.class, "shoot_l");
        shoot_main = hardwareMap.get(DcMotor.class, "shoot_main");
        lever = hardwareMap.get(Servo.class, "lever");

        shoot_top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_main.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot_main.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_main.setDirection(DcMotor.Direction.REVERSE);

        PIDController topShootController = new PIDController(shooterTuner_topP, shooterTuner_topI, shooterTuner_topD);
        PIDController mainShootController = new PIDController(shooterTuner_mainP, shooterTuner_mainI, shooterTuner_mainD);


        double driverOrientedHeading = 90;
        if (!PoseStorage.alianceIsBlue){
            driverOrientedHeading = -90;
        }

        PIDController tagLineUpController = new PIDController(1.2,0,0.05);
        PIDController driveXController = new PIDController(1.2,0,0.05);
        PIDController driveYController = new PIDController(1.2,0,0.05);


        double shootVel = 1700;
        boolean autoLineUpFirstLoop = true;
        int traySpun = 0;
        String ballPattern = PoseStorage.ballPattern;
        double distanceFromGoal = 55;
        aprilTagTimer.reset();
        boolean cameraExposerSet = false;
        double targetAngle = 0;
        boolean noTagYet = true;
        double totalDriveError = 20;
        boolean noTagsInView = true;

//        while (opModeInInit()){
//            if (!cameraExposerSet){
//                cameraExposerSet = camLibrary.setExposure();
//            }
//        }

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                try {
    //                if (!cameraExposerSet){
    //                    cameraExposerSet = camLibrary.setExposure();
    //                }
    //                telemetry.addData("CamStatus", camLibrary.checkForCamErrors());
                    topShootController.setPIDConstants(shooterTuner_topP, shooterTuner_topI, shooterTuner_topD);
                    mainShootController.setPIDConstants(shooterTuner_mainP, shooterTuner_mainI, shooterTuner_mainD);

                    driveXController.setPIDConstants(drivep, drivei, drived);
                    driveYController.setPIDConstants(drivep, drivei, drived);



                    Pose2d localizerPose = drive.localizer.getPose();

                    // Gets The Goals April Tag
                    detectedTag = null;

                    if (camLibrary.camera2IsStreaming()) {
                        if (aprilTagTimer.milliseconds() > 300) {
                            ArrayList<AprilTagDetection> freshDetections = (ArrayList<AprilTagDetection>) camLibrary.aprilTagProcessor.getFreshDetections();
                            if (freshDetections != null) {
                                noTagsInView = true;
                                for (AprilTagDetection detection : freshDetections) {
                                    if (detection.id == 20 || detection.id == 24) {
                                        noTagsInView = false;
                                        detectedTag = detection;
                                    }
                                }
                            }
                            aprilTagTimer.reset();
                        }
                    } else {
                        cameraCrashed = true;
                    }



//                    if (cameraCrashed){
//                        telemetry.addData("The Camera Crashed But the Program still works", "YAY");
//                        telemetry.addData("The Camera Crashed But the Program still works", "YAY");
//                        telemetry.addData("The Camera Crashed But the Program still works", "YAY");
//                        telemetry.addData("The Camera Crashed But the Program still works", "YAY");
//                        telemetry.addData("The Camera Crashed But the Program still works", "YAY");
//
//                    }
                    if (gamepad1.b){
                        cameraCrashed = false;
                    }


                    // Update the robot x and y with an estimet from the camera
                    if (detectedTag != null && Math.abs(totalDriveError) > 2) {
                        drive.localizer.setPose(new Pose2d(-detectedTag.robotPose.getPosition().x,-detectedTag.robotPose.getPosition().y, localizerPose.heading.toDouble()));
                    }

    //                distanceFromGoal = Math.sqrt(Math.pow(tagPose.x - localizerPose.position.x, 2) + Math.pow(tagPose.y - localizerPose.position.y, 2));
                    distanceFromGoal = Math.sqrt(Math.pow(goalPos.x - localizerPose.position.x, 2) + Math.pow(goalPos.y - localizerPose.position.y, 2));


                    // Calculate the shooters power
                    double topVelocity = topShootPowOverride;
                    double mainVelocity = mainShootPowOverride;
                    if (topShootPowOverride == 0) {
    //                    mainVelocity = 1130;
    //                    topVelocity = 1050;
    //                    if (detectedTag != null) {
    //                        mainVelocity = 0.738 * detectedTag.ftcPose.range + 1076.8;
    //                        topVelocity = 3.69 * detectedTag.ftcPose.range + 783.8;
                        mainVelocity = 0.95 * distanceFromGoal + 1044.5;
                        topVelocity = 4.76 * distanceFromGoal + 621.6;
    //                    }
                    }
                    telemetry.addData("mainVelocity", mainVelocity);
                    telemetry.addData("topVelocity", topVelocity);

                    double topShootPower = topVelocity*shooterTuner_topF+topShootController.update(topVelocity, ((DcMotorEx) shoot_top).getVelocity());
                    double mainShootPower = mainVelocity*shooterTuner_mainF+mainShootController.update(mainVelocity, ((DcMotorEx) shoot_main).getVelocity());



    //                double topShootPower = shooterTuner_topShootVelocity*shooterTuner_topF+topShootController.update(shooterTuner_topShootVelocity, ((DcMotorEx) shoot_top).getVelocity());
    //                double mainShootPower = shooterTuner_mainShootVelocity*shooterTuner_mainF+mainShootController.update(shooterTuner_mainShootVelocity, ((DcMotorEx) shoot_main).getVelocity());

    //                double shootersPower = shootersVelocity/2650+shootController.update(shootersVelocity, ((DcMotorEx) shoot_l).getVelocity());

                    int[] zoneColors = camLibrary.trayProcessor.getBallColors();
                    trayController.update(zoneColors);
    //                trayController.update();

                    if (gamepad1.a) {
                        if (gamepad1.aWasPressed()) {
                            shootTime.reset();
                            topShootController.resetTimer();
                            mainShootController.resetTimer();
                            trayController.setAutoSortTo(false);
                            trayController.setCode(ballPattern);
                            trayController.spinToShootReady();
//                            camLibrary.portal1.stopStreaming();

                        }

                        shoot_top.setPower(topShootPower);
                        shoot_main.setPower(mainShootPower);


                        if (shootTime.milliseconds() > 750) {
                            lever.setPosition(1);
                        }
                        if (shootTime.milliseconds() > 1500 && traySpun == 0 || shootTime.milliseconds() > 3000 && traySpun == 1 || shootTime.milliseconds() > 4500 && traySpun == 2) {
                            trayController.spinLeft(.3);
                            traySpun ++;
                        }
                    } else {
                        Drive_Speed = 0.7;
                        shoot_top.setPower(0);
                        shoot_main.setPower(0);
                        trayController.setAutoSortTo(true);
    //                    trayController.setAutoSortTo(false);
                        lever.setPosition(.8);
                        traySpun = 0;
//                        if (gamepad1.aWasReleased()){
//                            camLibrary.portal1.resumeStreaming();
//                        }

                    }

                    if (zoneColors[0] == 0 || zoneColors[1] == 0 || zoneColors[2] == 0) {
                        pickUpTimer.reset();
                    }
//                    if (gamepad1.xWasReleased()){
//                        camLibrary.portal2.resumeStreaming();
//                    }

                    if (gamepad1.x) {
//                        if (gamepad1.xWasPressed()){
//                            camLibrary.portal2.stopStreaming();
//                        }
                        if (pickUpTimer.milliseconds() > 250) {
                            intake.setPower(-1);
                        } else {
                            intake.setPower(1);
                        }
                    } else if (gamepad1.dpad_down) {
                        intake.setPower(-1);
                    } else if (gamepad1.dpad_up) {
                        intake.setPower(1);
                    } else {
                        intake.setPower(0);
                    }
                    if (gamepad1.leftBumperWasPressed()) {
                        trayController.spinLeft();
                    } else if (gamepad1.rightBumperWasPressed()) {
                        trayController.spinRight();
                    }


    //                TelemetryPacket packet = new TelemetryPacket();
                    shootVel = 1720;
                    if (gamepad1.left_trigger + gamepad1.right_trigger > 1.1 || gamepad1.y|| gamepad1.b) {
    //                    if (gamepad1.bWasPressed()){
    //                        camLibrary.portal2.resumeStreaming();
    //                    }

                        if (camLibrary.portal2 != null) {
                            if (detectedTag != null){
    //                        This is what the qualifier version. was i made it positive to get the state to work
    //                        turnError = Math.toRadians(-detectedTag.ftcPose.bearing);
                                double barringOffset = barringOffsetOverride;
                                if (barringOffsetOverride == 0){
                                    barringOffset = 0.1565 * localizerPose.position.x -5.20615;
                                }

    //                        telemetry.addData("barringOffset", barringOffset);
    //                        telemetry.addData("detectedTag.ftcPose.bearing)", detectedTag.ftcPose.bearing);



                                double newTargetAngle = localizerPose.heading.toDouble()-Math.PI*.5+Math.toRadians(detectedTag.ftcPose.bearing-barringOffset);
                                if (Math.abs(newTargetAngle - targetAngle) > Math.PI*.02) {
                                    targetAngle = newTargetAngle;
                                }

    //                        noTagYet = false;
                            } else if (/*noTagsInView&& */camLibrary.portal2.getCameraState() == VisionPortal.CameraState.STREAMING) {
                                if (Math.abs(xPower) + Math.abs(yPower) > .25 || noTagsInView){
                                    targetAngle = Math.atan2(tagPose.y - localizerPose.position.y, tagPose.x - localizerPose.position.x);
                                }
                            }
                        }

                        telemetry.addData("targetAngle", targetAngle);

                        turnError = (targetAngle - localizerPose.heading.toDouble())+Math.PI*.5;
                        if (turnError > Math.PI){
                            turnError -= Math.PI*2;
                        }
                        telemetry.addData("turnError", turnError);

                        if (autoLineUpFirstLoop) {
                            tagLineUpController.resetTimer();
                        }
                        autoLineUpFirstLoop = false;
                        turnPower = tagLineUpController.update(turnError, 0);
                        if (turnPower > turnMax) {
                            turnPower = turnMax;
                        } else if (turnPower < -turnMax) {
                            turnPower = -turnMax;
                        }
                    } else {
    //                    if (gamepad1.bWasReleased()) {
    //                        camLibrary.portal2.stopStreaming();
    //                    }
                        turnError = Math.PI;
                        noTagYet = true;
                        autoLineUpFirstLoop = true;
                        turnPower = gamepad1.left_trigger - gamepad1.right_trigger;
                    }
    //                if (gamepad1.left_trigger + gamepad1.right_trigger > 1.1 || gamepad1.y) {
    //                    turnError = 0;
    //
    //                    if (detectedTag != null){
    ////                        This is what the qualifier version. was i made it positive to get the state to work
    ////                        turnError = Math.toRadians(-detectedTag.ftcPose.bearing);
    //                        double barringOffset = barringOffsetOverride;
    //                        if (barringOffsetOverride == 0){
    //                            barringOffset = 0.1565 * localizerPose.position.x -5.20615;
    //                        }
    //
    //                        telemetry.addData("barringOffset", barringOffset);
    //                        telemetry.addData("detectedTag.ftcPose.bearing)", detectedTag.ftcPose.bearing);
    //
    //                        turnError = Math.toRadians(detectedTag.ftcPose.bearing-barringOffset);
    ////                        turnError = Math.toRadians(detectedTag.ftcPose.bearing);
    //                    }
    //
    //                    if (turnError == 0) {
    //                        double angleToTarget = Math.atan2(tagPose.y - localizerPose.position.y, tagPose.x - localizerPose.position.x);
    //                        turnError = (angleToTarget - localizerPose.heading.toDouble())+Math.PI*.5;
    //                        if (turnError > Math.PI){
    //                            turnError -= Math.PI*2;
    //                        }
    //                    }
    ////                    if (turnError == 0) {
    ////                        double angleToTarget = Math.atan2(goalPos.y - localizerPose.position.y, goalPos.x - localizerPose.position.x);
    ////                        turnError = (angleToTarget - localizerPose.heading.toDouble())+Math.PI*.5;
    ////                        if (turnError > Math.PI){
    ////                            turnError -= Math.PI*2;
    ////                        }
    ////                    }
    //
    //
    //
    ////                        Vector2d tagToTarget = goalPos.minus(tagPose);
    ////                        Vector2d robotToTag = tagPose.minus(localizerPose.position);
    ////                        if (detectedTag != null){
    ////                            double bearing = Math.toRadians(-detectedTag.ftcPose.bearing);
    ////                            double heading = localizerPose.heading.toDouble()+180;
    ////                            if (heading > 180){
    ////                                heading -= 360;
    ////                            }
    ////                            robotToTag = new Vector2d(distanceFromGoal*Math.cos(heading+bearing),distanceFromGoal*Math.sin(heading+bearing));
    ////                            telemetry.addData("bearing", bearing);
    ////                        }
    ////                        Vector2d robotToGoal = robotToTag.plus(tagToTarget);
    ////                        double angleToTarget = Math.atan2(robotToGoal.y, robotToGoal.x);
    ////                        turnError = (angleToTarget - localizerPose.heading.toDouble())+Math.PI;
    ////                        if (turnError > Math.PI){
    ////                            turnError -= Math.PI*2;
    ////                        }
    //
    //                    if (autoLineUpFirstLoop) {
    //                        tagLineUpController.resetTimer();
    ////                        for (AprilTagDetection detection : camLibrary.aprilTagProcessor.getDetections()) {
    ////                            if (detection.id == 20 || detection.id == 24) {
    ////                                bearingError = Math.toRadians(detection.ftcPose.bearing);
    ////                                turnError = bearingError;
    ////                                oldRobotHeading = drive.localizer.getPose().heading.toDouble();
    ////                            }
    ////                        }
    ////                    } else {
    ////                        for (AprilTagDetection detection : camLibrary.aprilTagProcessor.getDetections()) {
    ////                            if (detection.id == 20 || detection.id == 24) {
    ////                                double newBearingError = Math.toRadians(detection.ftcPose.bearing);
    ////                                if(bearingError != newBearingError){
    ////                                    double changeInBearingError = newBearingError - bearingError;
    ////                                    double newRobotHeading = drive.localizer.getPose().heading.toDouble();
    ////                                    double changeInHeading = newRobotHeading - oldRobotHeading;
    ////                                    double changeInError = changeInBearingError - changeInHeading;
    ////                                    turnError = bearingError + changeInHeading + changeInError * DashboardVariables.turn_cam_efect;
    ////
    ////                                    bearingError = newBearingError;
    ////                                    oldRobotHeading = newRobotHeading;
    ////                                }
    //
    //
    //
    ////                            Pose2d roadrunnerPose = drive.localizer.getPose();
    ////                            Pose3D cameraPose = detection.robotPose;
    ////                            shootVel = 7 * detection.ftcPose.range + 1230;
    //////                            double error_x = -(roadrunnerPose.position.x + cameraPose.getPosition().x);
    //////                            double error_y = -(roadrunnerPose.position.y + cameraPose.getPosition().y);
    //////                                drive.localizer.setPose(new Pose2d(roadrunnerPose.position.x-error_x*.05,roadrunnerPose.position.y-error_y*.05, roadrunnerPose.heading.toDouble()+((cameraPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5)-roadrunnerPose.heading.toDouble())*.05));
    ////                            drive.localizer.setPose(new Pose2d(roadrunnerPose.position.x, roadrunnerPose.position.y, roadrunnerPose.heading.toDouble() + ((cameraPose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI * .5) - roadrunnerPose.heading.toDouble()) * .05));
    ////                            }
    ////                        }
    //                    }
    //                    autoLineUpFirstLoop = false;
    //                    turnPower = tagLineUpController.update(turnError, 0);
    //                    if (turnPower > turnMax) {
    //                        turnPower = turnMax;
    //                    } else if (turnPower < -turnMax) {
    //                        turnPower = -turnMax;
    //                    }
    //                } else {
    //                    autoLineUpFirstLoop = true;
    //                    turnPower = gamepad1.left_trigger - gamepad1.right_trigger;
    //                }

                    if (gamepad1.back){
                        if (PoseStorage.alianceIsBlue) {
                            driverOrientedHeading = Math.PI*.5;
                            drive.localizer.setPose(new Pose2d(0,0, Math.PI*.5));
                        } else {
                            driverOrientedHeading = -Math.PI*.5;
                            drive.localizer.setPose(new Pose2d(0,0, -Math.PI*.5));
                        }
                    }

                    double heading = localizerPose.heading.toDouble();
                    if (gamepad1.start){
                        driverOrientedHeading = heading;
                    }
                    heading -= driverOrientedHeading;

                    if (gamepad1.y) {
                        double xTarget = -52;
                        double yTarget = -15;
                        double xError = xTarget - localizerPose.position.x;
                        telemetry.addData("XERROR", xError);
                        double yError = yTarget - localizerPose.position.y;

                        totalDriveError = Math.sqrt(Math.pow(xError,2)+Math.pow(yError,2));

                        if (gamepad1.yWasPressed()) {
                            driveXController.resetTimer();
                            driveYController.resetTimer();
                        }
                        xPower = driveXController.update(xError, 0);
                        if (xPower > 0){
                            xPower += drivef;
                        } else if (xPower < 0) {
                            xPower -= drivef;
                        }
                        if (xPower > drivemax){
                            xPower = drivemax;
                        } else if (xPower < -drivemax) {
                            xPower = -drivemax;
                        }

                        yPower = -driveYController.update(yError, 0);
                        if (yPower > 0){
                            yPower += drivef;
                        } else if (yPower < 0) {
                            yPower -= drivef;
                        }
                        if (yPower > drivemax){
                            yPower = drivemax;
                        } else if (yPower < -drivemax) {
                            yPower = -drivemax;
                        }
                        barringOffsetOverride = -3.5;
                        mainShootPowOverride = 1140;
                        topShootPowOverride = 1425;
                    } else {
                        totalDriveError = 15;
                        barringOffsetOverride = 0;
                        mainShootPowOverride = 0;
                        topShootPowOverride = 0;
                        yPower = gamepad1.left_stick_y;
                        xPower = gamepad1.left_stick_x;
                    }

                    double orientedXPower = xPower * Math.cos(heading) - yPower * Math.sin(heading);
                    double orientedYPower = yPower* Math.cos(heading) + xPower * Math.sin(heading);

                    front_l.setPower(((orientedYPower - orientedXPower) + turnPower) * -1);// * -Drive_Speed);
                    back_l.setPower(((orientedYPower + orientedXPower) + turnPower) * -1);
                    front_r.setPower((orientedYPower + orientedXPower - turnPower) * -1);
                    back_r.setPower(((orientedYPower - orientedXPower) - turnPower) * -1);

                    drive.updatePoseEstimate();

                    // Put loop blocks here.

    //                if (Math.abs(magPos-intake.getCurrentPosition())<200 && ((DcMotorEx) shoot_l).getVelocity() < 500) {
    //                    if (zoneColors[0] == 0 || zoneColors[1] == 0 || zoneColors[2] == 0) {
    //                        if (zoneColors[1] == 0) {
    //                            if ((zoneColors[4] == 0)){
    //                                if (zoneColors[0] > 0){
    //                                    magPos -= 2731;
    //                                    servoController.resetTimer();
    //                                } else if (zoneColors[2]  > 0) {
    //                                    magPos += 2731;
    //                                    servoController.resetTimer();
    //                                }
    //                            } else {
    //                                if (zoneColors[2]  > 0) {
    //                                    magPos += 2731;
    //                                    servoController.resetTimer();
    //                                } else if (zoneColors[0] > 0){
    //                                    magPos -= 2731;
    //                                    servoController.resetTimer();
    //                                }
    //                            }
    //                        } else if (zoneColors[3] > 0 && zoneColors[2] == 0) {
    //                            magPos -= 2731;
    //                            servoController.resetTimer();
    //                        } else if (zoneColors[4] > 0 && zoneColors[0] == 0) {
    //                            magPos += 2731;
    //                            servoController.resetTimer();
    //                        }
    //                    }
    //                }

    //                if (gamepad1.dpad_down){
    //                    for(AprilTagDetection detection: camLibrary.aprilTagProcessor.getDetections()){
    //                        if (detection.id == 20 || detection.id == 24){
    //                            drive.localizer.setPose(new Pose2d(-detection.robotPose.getPosition().x,-detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5));
    //                        }
    //                    }
    //                }
                    if (gamepad2.bWasPressed()){
                        ballPattern = ballPattern.substring(1) + "P";
                        trayController.setCode(ballPattern);
                    } else if (gamepad2.aWasPressed()) {
                        ballPattern = ballPattern.substring(1) + "G";
                        trayController.setCode(ballPattern);
                    }
                    if (gamepad2.start){
                        PoseStorage.alianceIsBlue = true;
                        tagPose = new Vector2d(59, 56);
                        goalPos = new Vector2d(69, 69);
                    } else if (gamepad2.back) {
                        PoseStorage.alianceIsBlue = false;
                        tagPose = new Vector2d(59, -56);
                        goalPos = new Vector2d(69, -69);
                    }


                    telemetry.addData("Ball Code", ballPattern);
                    telemetry.addData("", "");
                    telemetry.addData("topShootTargetVel", shooterTuner_topShootVelocity);
                    telemetry.addData("topShootCurrentVel", ((DcMotorEx) shoot_top).getVelocity());
                    telemetry.addData("", "");
                    telemetry.addData("mainShootTargetVel", shooterTuner_mainShootVelocity);
                    telemetry.addData("mainShootCurrentVel", ((DcMotorEx) shoot_main).getVelocity());
                    telemetry.addData("", "");
                    telemetry.addData("Zone 1", zoneColors[0]);
                    telemetry.addData("Zone 2", zoneColors[1]);
                    telemetry.addData("Zone 3", zoneColors[2]);
                    telemetry.addData("Zone 4", zoneColors[3]);
                    telemetry.addData("Zone 5", zoneColors[4]);

                    telemetry.addData("turnError", turnError);
                    telemetry.addData("turnPower", turnPower);
                    //todo
    //                telemetry.addData("action: ", drive_action);
    //                telemetry.addData("X: ", drive.localizer.getPose().position.x);
    //                telemetry.addData("Y: ", drive.localizer.getPose().position.y);
    //                telemetry.addData("heading: ", drive.localizer.getPose().heading.toDouble());
                    if (detectedTag != null) {
                        telemetry.addData("id", detectedTag.id);
                        if (detectedTag.robotPose != null) {
                            telemetry.addData("robotPose", detectedTag.robotPose.getPosition().toString());
                            telemetry.addData("robotAngle", detectedTag.robotPose.getOrientation().toString());
                        }
                    }
                    telemetry.addData("angle", localizerPose.heading.toDouble());
                    telemetry.addData("x", localizerPose.position.x);
                    telemetry.addData("y", localizerPose.position.y);
    //                telemetry.addData("target and actual vel", shootVel + " / " + ((DcMotorEx) shoot_l).getVelocity());
                    telemetry.update();



    //                if (gamepad1.start) {
    //                    drive.localizer.setPose(new Pose2d(0, 0, 0));
    //                }
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
        }
        if (camLibrary.portal1 != null){
            camLibrary.portal1.close();
        }
        if (camLibrary.portal2 != null){
            camLibrary.portal2.close();
        }
    }
}