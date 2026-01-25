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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp(name="Cam Driver Qualifiers", group = "B")
public class OpModeWithPic extends LinearOpMode {
    MecanumDrive drive;
    private DcMotor shoot_l;
    //    private DcMotor shoot_r;
    private DcMotor arm;
    private DcMotor back_l;
    private DcMotor front_r;
    private DcMotor back_r;
    private DcMotor intake;
    //    private CRServo magazen;
    private DcMotor front_l;
    private List<Action> runningActions = new ArrayList<>();
    private double turnError = 0;
    private double bearingError = 0;
    private double turnPower = 0;
    private double oldRobotHeading = 0;
    Vector2d tagPose = new Vector2d(59, 56);
    Vector2d goalPos = new Vector2d(69, 69);
    AprilTagDetection detectedTag = null;
    private ElapsedTime shootTime = new ElapsedTime();
    private ElapsedTime cameTime = new ElapsedTime();

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
//        if (PoseStorage.currentPose.heading.toDouble() == 0 && PoseStorage.currentPose.position.y == 0) {
//            if (PoseStorage.alianceIsBlue){
//
//            }
//        }
        Pose2d initialPose = PoseStorage.currentPose;

        drive = new MecanumDrive(hardwareMap, initialPose);
        TrayController trayController = new TrayController(hardwareMap);
        trayController.setAutoSortTo(true);
        TwoCamLibrary camLibrary = new TwoCamLibrary(hardwareMap);
        ElapsedTime myElapsedTime;
        double Drive_Speed = .7;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (!PoseStorage.alianceIsBlue){
            tagPose = new Vector2d(59, -56);
            goalPos = new Vector2d(69, -69);
        }

//        List magPosList;
//        int magPos;
//        boolean left_bumper_down = false;
//        boolean right_bumper_down = false;

        Action drive_action = null;


        shoot_l = hardwareMap.get(DcMotor.class, "shoot_l");
//        shoot_r = hardwareMap.get(DcMotor.class, "shoot_r");
        arm = hardwareMap.get(DcMotor.class, "arm");
        back_l = hardwareMap.get(DcMotor.class, "back_l");
        front_r = hardwareMap.get(DcMotor.class, "front_r");
        back_r = hardwareMap.get(DcMotor.class, "back_r");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        magazen = hardwareMap.get(CRServo.class, "magazen");
        front_l = hardwareMap.get(DcMotor.class, "front_l");

        // Put initialization blocks here.
        shoot_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shoot_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
//        shoot_r.setDirection(DcMotor.Direction.REVERSE);
//        front_l.setDirection(DcMotor.Direction.FORWARD);
//        back_l.setDirection(DcMotor.Direction.REVERSE);
//        front_r.setDirection(DcMotor.Direction.REVERSE);
//        back_r.setDirection(DcMotor.Direction.REVERSE);

        double driverOrientedHeading = 90;
        if (!PoseStorage.alianceIsBlue){
            driverOrientedHeading = -90;
        }

        PIDController servoController = new PIDController(0.001,0,0.000025);
        PIDController tagLineUpController = new PIDController(1.2,0,0.05);
        PIDController shootController = new PIDController(0.006, 0, 0);


//        magPos = 0;
        double shootVel = 1700;
        boolean autoLineUpFirstLoop = true;
        boolean traySpun = false;
        String ballPattern = PoseStorage.ballPattern;
        double distanceFromGoal = 55;

        waitForStart();
        if (opModeIsActive()) {
            cameTime.startTime();
            // Put run blocks here.
            while (opModeIsActive()) {
                if (cameTime.milliseconds() > 4000){
                    camLibrary.trayProcessor.setSaveMat(true, "drive_");
                    cameTime.reset();
                }

                Pose2d localizerPose = drive.localizer.getPose();

                // Gets The Goals April Tag
                detectedTag = null;
                for (AprilTagDetection detection : camLibrary.aprilTagProcessor.getDetections()) {
                    if (detection.id == 20 || detection.id == 24) {
                        detectedTag = detection;
                    }
                }

                // Update the robot x and y with an estimet from the camera
                if (detectedTag != null) {
                    drive.localizer.setPose(new Pose2d(-detectedTag.robotPose.getPosition().x,-detectedTag.robotPose.getPosition().y, localizerPose.heading.toDouble()));
                }


                distanceFromGoal = Math.sqrt(Math.pow(tagPose.x - localizerPose.position.x, 2) + Math.pow(tagPose.y - localizerPose.position.y, 2));

                // Calculate the shooters power
                double shootersVelocity = 7 * distanceFromGoal + 1300;

                double shootersPower = shootersVelocity/2650+shootController.update(shootersVelocity, ((DcMotorEx) shoot_l).getVelocity());

                if (gamepad1.a) {
                    if (gamepad1.aWasPressed()) {
                        shootTime.reset();
                        shootController.resetTimer();
                        trayController.setAutoSortTo(false);
                        traySpun = false;
                        trayController.setCode(ballPattern);
                    }

                    shoot_l.setPower(shootersPower);

                    if (shootTime.milliseconds() < 750) {
                        if (!traySpun){
                            trayController.spinToNextInCode();
                        }
                        traySpun = true;
                    } else if (shootTime.milliseconds() < 1100) {
                        arm.setPower(1);
                    } else if (shootTime.milliseconds() < 1400) {
                        arm.setPower(0.0);
                    } else {
                        arm.setPower(0);
                        shootTime.reset();
                        traySpun = false;
                    }
                } else if (gamepad1.y) {
                    Drive_Speed = 0.4;
                    if (gamepad1.yWasPressed()) {
                        trayController.setAutoSortTo(false);
                        shootController.resetTimer();
                    }

                    shoot_l.setPower(shootersPower);
//                    ((DcMotorEx) shoot_r).setVelocity(1600);
                } else {
                    Drive_Speed = 0.7;
                    ((DcMotorEx) shoot_l).setPower(0);
                    trayController.setAutoSortTo(true);
                }
                if (gamepad1.x) {
                    intake.setPower(1);
                } else if (gamepad1.dpad_down) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }
                if (gamepad1.b) {
                    arm.setPower(1);
                } else if (!gamepad1.a) {
                    arm.setPower(0);
                }
                if (gamepad1.leftBumperWasPressed()) {
                    trayController.spinLeft();
                } else if (gamepad1.rightBumperWasPressed()) {
                    trayController.spinRight();
                }
                int[] zoneColors = camLibrary.trayProcessor.getBallColors();
                trayController.update(zoneColors);

//                TelemetryPacket packet = new TelemetryPacket();
                shootVel = 1720;
                if (gamepad1.left_trigger + gamepad1.right_trigger > 1.1) {
                    turnError = 0;

                    if (detectedTag != null){
                        turnError = Math.toRadians(-detectedTag.ftcPose.bearing);
                    }

                    if (turnError == 0) {
                        double angleToTarget = Math.atan2(tagPose.y - localizerPose.position.y, tagPose.x - localizerPose.position.x);
                        turnError = (angleToTarget - localizerPose.heading.toDouble())+Math.PI;
                        if (turnError > Math.PI){
                            turnError -= Math.PI*2;
                        }
                    }

//                    if (turnError == 0) {


////                        Vector2d tagToTarget = goalPos.minus(tagPose);
//                        Vector2d robotToTag = tagPose.minus(localizerPose.position);
//                        if (detectedTag != null){
//                            double bearing = Math.toRadians(-detectedTag.ftcPose.bearing);
//                            double heading = localizerPose.heading.toDouble()+180;
//                            if (heading > 180){
//                                heading -= 360;
//                            }
//                            robotToTag = new Vector2d(distanceFromGoal*Math.cos(heading-bearing),distanceFromGoal*Math.sin(heading-bearing));
//                            telemetry.addData("bearing", bearing);
//                        }
////                        Vector2d robotToGoal = robotToTag.plus(tagToTarget);
//                        double angleToTarget = Math.atan2(robotToGoal.y, robotToGoal.x);
//                        turnError = (angleToTarget - localizerPose.heading.toDouble())+Math.PI;
//                        if (turnError > Math.PI){
//                            turnError -= Math.PI*2;
//                        }
//                        turnError = Math.toRadians(-detectedTag.ftcPose.bearing);
//                    }
                    if (autoLineUpFirstLoop) {
                        tagLineUpController.resetTimer();
//                        for (AprilTagDetection detection : camLibrary.aprilTagProcessor.getDetections()) {
//                            if (detection.id == 20 || detection.id == 24) {
//                                bearingError = Math.toRadians(detection.ftcPose.bearing);
//                                turnError = bearingError;
//                                oldRobotHeading = drive.localizer.getPose().heading.toDouble();
//                            }
//                        }
//                    } else {
//                        for (AprilTagDetection detection : camLibrary.aprilTagProcessor.getDetections()) {
//                            if (detection.id == 20 || detection.id == 24) {
//                                double newBearingError = Math.toRadians(detection.ftcPose.bearing);
//                                if(bearingError != newBearingError){
//                                    double changeInBearingError = newBearingError - bearingError;
//                                    double newRobotHeading = drive.localizer.getPose().heading.toDouble();
//                                    double changeInHeading = newRobotHeading - oldRobotHeading;
//                                    double changeInError = changeInBearingError - changeInHeading;
//                                    turnError = bearingError + changeInHeading + changeInError * DashboardVariables.turn_cam_efect;
//
//                                    bearingError = newBearingError;
//                                    oldRobotHeading = newRobotHeading;
//                                }



//                            Pose2d roadrunnerPose = drive.localizer.getPose();
//                            Pose3D cameraPose = detection.robotPose;
//                            shootVel = 7 * detection.ftcPose.range + 1230;
////                            double error_x = -(roadrunnerPose.position.x + cameraPose.getPosition().x);
////                            double error_y = -(roadrunnerPose.position.y + cameraPose.getPosition().y);
////                                drive.localizer.setPose(new Pose2d(roadrunnerPose.position.x-error_x*.05,roadrunnerPose.position.y-error_y*.05, roadrunnerPose.heading.toDouble()+((cameraPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5)-roadrunnerPose.heading.toDouble())*.05));
//                            drive.localizer.setPose(new Pose2d(roadrunnerPose.position.x, roadrunnerPose.position.y, roadrunnerPose.heading.toDouble() + ((cameraPose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI * .5) - roadrunnerPose.heading.toDouble()) * .05));
//                            }
//                        }
                    }
                    autoLineUpFirstLoop = false;
                    turnPower = tagLineUpController.update(turnError, 0);
                } else {
                    autoLineUpFirstLoop = true;
                    turnPower = gamepad1.left_trigger - gamepad1.right_trigger;
                }
//                    if(gamepad1.dpadUpWasPressed() && !drive.extraCorrection) {
////                        for(AprilTagDetection detection: camLibrary.aprilTagProcessor.getDetections()){
////                            if (detection.id == 20 || detection.id == 24){
////                                Pose2d roadrunnerPose = drive.localizer.getPose();
////                                Pose3D cameraPose = detection.robotPose;
////                                double error_x = -(roadrunnerPose.position.x + cameraPose.getPosition().x);
////                                double error_y = -(roadrunnerPose.position.y + cameraPose.getPosition().y);
//////                                drive.localizer.setPose(new Pose2d(roadrunnerPose.position.x-error_x,roadrunnerPose.position.y-error_y, cameraPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5));
////
////
////                                drive.localizer.setPose(new Pose2d(roadrunnerPose.position.x,roadrunnerPose.position.y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5));
////                            }
////                        }
//
//
//                        Pose2d current = drive.localizer.getPose();
//                        Pose2d target = new Pose2d(10, 10, Math.toRadians(45));
//                        double delta_x = target.position.x - current.position.x;
//                        double delta_y = target.position.y - current.position.y;
//                        double delta_a = target.heading.toDouble() - current.heading.toDouble();
//                        if (delta_x!=0||delta_y!=0|delta_a!=0) {
//                            double tangent = Math.atan2(delta_y, delta_x);
//
//                            drive_action = drive.actionBuilder(current)
//                                    .turnTo(Math.toRadians(45))
////                                    .setTangent(tangent)
//////                                    .lineToXLinearHeading(target.position.x,)
////                                    .splineToLinearHeading(
////                                            target,
////                                            tangent,
////                                            new AngularVelConstraint(Math.toRadians(45))
////                                    )
////                                            new MinVelConstraint(
////                                                    Arrays.asList(
////                                                            new TranslationalVelConstraint(10),
////                                                            new AngularVelConstraint(1)
////                                                    )),
////                                            new ProfileAccelConstraint(-2, 2)
////                                            )
//                                    .build();
//                        }
//                        drive.extraCorrection = true;
//                    } else {
//
////                        for(AprilTagDetection detection: camLibrary.aprilTagProcessor.getDetections()){
////                            if (detection.id == 20 || detection.id == 24){
////                                drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x,drive.localizer.getPose().position.y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5));
////
//////                                drive.localizer.setPose(new Pose2d(detection.robotPose.getPosition().x,detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5));
////                            }
////                        }
////                        for(AprilTagDetection detection: camLibrary.aprilTagProcessor.getDetections()){
////                            if (detection.id == 20 || detection.id == 24){
////                                drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x+(drive.localizer.getPose().position.x-(-detection.robotPose.getPosition().x))*.05,drive.localizer.getPose().position.y-(drive.localizer.getPose().position.y-(-detection.robotPose.getPosition().y))*.05, drive.localizer.getPose().heading.toDouble()-(drive.localizer.getPose().heading.toDouble()-(detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)-Math.PI*.5))*.05));
////                            }
////                        }
//                    }
//
//
//
//                    if(drive_action != null){
//                        drive_action.preview(packet.fieldOverlay());
//                        if(!drive_action.run(packet)){
//                            drive_action = null;
//                        }
//                    }
//                } else {
//                    drive.extraCorrection = false;
//                    drive_action = null;

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

                double sticky = gamepad1.left_stick_y * Math.cos(heading) + gamepad1.left_stick_x * Math.sin(heading);
                double stickx = gamepad1.left_stick_x * Math.cos(heading) - gamepad1.left_stick_y * Math.sin(heading);

                front_l.setPower(((sticky - stickx) + turnPower) * -Drive_Speed);
                back_l.setPower(((sticky + stickx) + turnPower) * -Drive_Speed);
                front_r.setPower((sticky + stickx - turnPower) * -Drive_Speed);
                back_r.setPower(((sticky - stickx) - turnPower) * -Drive_Speed);

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
                telemetry.addData("target_Vel", 7 * 55 + 1230);
                telemetry.addData("Current_vel", ((DcMotorEx) shoot_l).getVelocity());
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
                for(AprilTagDetection detection: camLibrary.aprilTagProcessor.getDetections()){
                    telemetry.addData("id", detection.id);
                    if (detection.robotPose != null) {
                        telemetry.addData("robotPose", detection.robotPose.getPosition().toString());
                    }
                }
                telemetry.addData("angle", localizerPose.heading.toDouble());
                telemetry.addData("x", localizerPose.position.x);
                telemetry.addData("y", localizerPose.position.y);
                telemetry.addData("target and actual vel", shootVel + " / " + ((DcMotorEx) shoot_l).getVelocity());
                telemetry.update();



//                if (gamepad1.start) {
//                    drive.localizer.setPose(new Pose2d(0, 0, 0));
//                }
            }
        }
    }
}