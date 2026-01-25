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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name = "DriveForward", group = "A")
public final class AutoRed2 extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    MecanumDrive drive;
    private ElapsedTime shootTime = new ElapsedTime();
    int magPos = 2;
    int ballNumber = 0;
    int[] ballPattern = {1,2,3};

    private DcMotor shoot_l;
    private DcMotor shoot_r;
    private DcMotor arm;
    private DcMotor intake;
    private CRServo magazen;
    int magTargetPos = 0;
    int[] magPosList  = {0, 2731, 2731*2, 2731*3};
    @Override
    public void runOpMode() throws InterruptedException {

        initAprilTag();

        ElapsedTime myElapsedTime;
        double Drive_Speed;



        boolean left_bumper_down = false;
        boolean right_bumper_down = false;

        shoot_l = hardwareMap.get(DcMotor.class, "shoot_l");
        shoot_r = hardwareMap.get(DcMotor.class, "shoot_r");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        magazen = hardwareMap.get(CRServo.class, "magazen");

        // Put initialization blocks here.
        shoot_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        shoot_r.setDirection(DcMotor.Direction.REVERSE);

        Pose2d initialPose = new Pose2d(0, 0, 0);


        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder driveToTag = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.75)
                .splineToConstantHeading(new Vector2d(30, 30), -Math.PI*.75);
        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(new Pose2d(30, 30, -Math.PI/180 * 36))
                .setTangent(Math.PI*.25)
                .splineToSplineHeading(new Pose2d(30, 25, Math.PI*.25), Math.PI*.25);
        TrajectoryActionBuilder driveToPickup = drive.actionBuilder(new Pose2d(45, 40, Math.PI*.25))
                .setTangent(-Math.PI*.75)
                .splineToSplineHeading(new Pose2d(12, 12, Math.PI*.5), Math.PI*.25)
                .setTangent(Math.PI*.5)
                .splineToConstantHeading(new Vector2d(12, 48), Math.PI*.5)
                .setTangent(Math.PI*.0)
                .splineToSplineHeading(new Pose2d(45, 40, Math.PI*.25), Math.PI*0);
        TrajectoryActionBuilder driveToPickup2 = drive.actionBuilder(new Pose2d(45, 40, Math.PI*.25))
                .setTangent(-Math.PI*.75)
                .splineToSplineHeading(new Pose2d(-12, 12, Math.PI*.5), Math.PI*.25)
                .setTangent(Math.PI*.5)
                .splineToConstantHeading(new Vector2d(-12, 48), Math.PI*.5)
                .setTangent(Math.PI*.0)
                .splineToSplineHeading(new Pose2d(45, 40, Math.PI*.25), Math.PI*0);

        TrajectoryActionBuilder driveToPark = drive.actionBuilder(initialPose)
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(30, 0), Math.PI);

        Action driveToTagAction = driveToTag.build();
        Action driveToShootAction = driveToShoot.build();
        Action driveToPickupAction = driveToPickup.build();
        Action driveToPickup2Action = driveToPickup2.build();
        Action driveToParkAction = driveToPark.build();

//        while (opModeInInit()) {
//            pass;
//        }

        telemetry.addData("ready to start", "true");
        telemetry.update();

        waitForStart();
        MagazenControler magControler = new MagazenControler();
        magTargetPos = 0;

        NonDrivingActions actions = new NonDrivingActions();

        Actions.runBlocking(
                new SequentialAction(
                        driveToParkAction
//                        driveToTagAction,
//                        actions.GetTagId(),
//                        driveToShootAction,
//                        actions.Shoot(3),
//                        actions.turnOnIntake(),
//                        driveToPickupAction,
//                        actions.turnOffIntake()
//                        actions.Shoot(2),
//                        actions.turnOnIntake(),
//                        driveToPickup2Action,
//                        actions.turnOffIntake(),
//                        actions.Shoot(1),
//                        driveToParkAction
                )
        );
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {



    }   // end method telemetryAprilTag()
    class NonDrivingActions {
        public Action turnOnIntake() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intake.setPower(1);
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

        public Action Shoot(int endingPos) {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (ballNumber == 0) {
                        shootTime.reset();
                        ((DcMotorEx) shoot_l).setVelocity(1070);
                        ((DcMotorEx) shoot_r).setVelocity(1070);
                        magPos = ballPattern[0];
                        ballNumber = 1;
                    }
                    int spinTimeChange = 0;
//                    if (ballNumber > 1) {
//                        if (Math.abs(magPos-ballPattern[ballNumber-2])==1){
//                            spinTimeChange = 200;
//                        }
//                    }
                    if (shootTime.milliseconds() < 500-spinTimeChange) {
                        magTargetPos = magPosList[magPos-1];
                        telemetry.addData("magPos",magPos);
                        telemetry.addData("magPosList",magPosList[magPos-1]);
                        telemetry.update();
                    } else if (shootTime.milliseconds() < 1050-spinTimeChange) {
                        arm.setPower(1);
                    } else if (shootTime.milliseconds() < 1450-spinTimeChange) {
                        arm.setPower(0.01);
                    } else {
                        arm.setPower(0);
                        shootTime.reset();
                        if (ballNumber < ballPattern.length){
                            magPos = ballPattern[ballNumber];
                        }
                        ballNumber += 1;
                    }

                    if (ballNumber > 3) {
                        ((DcMotorEx) shoot_l).setVelocity(0);
                        ((DcMotorEx) shoot_r).setVelocity(0);
                        ballNumber = 0;
                        magTargetPos = magPosList[endingPos-1];
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }
        public Action GetTagId() {
            return new Action() {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {

                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    // Step through the list of detections and display info for each one.
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.ftcPose.yaw > 0) {
                            int tagId = detection.id;
                            if (tagId == 21){
                                ballPattern = new int[]{1, 2, 3};
                            } else if (tagId == 22){
                                ballPattern = new int[]{3, 1, 2};
                            } else {
                                ballPattern = new int[]{3, 2, 1};
                            }
                        }
                    }   // end for() loop
                    telemetry.addData("x",drive.localizer.getPose().position.x);
                    telemetry.addData("y",drive.localizer.getPose().position.y);
                    telemetry.addData("test",ballPattern[0]+", "+ballPattern[1]+", "+ballPattern[2]);
                    telemetry.update();
                    if (true) {
                        visionPortal.close();
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }
    }
    class MagazenControler extends Thread{
        @Override
        public void run() {
            PIDController servoController = new PIDController(0.0015, 0, 0);
            while (opModeIsActive()) {
                magazen.setPower(servoController.update(magPos, intake.getCurrentPosition()));
            }
        }
    }
}

