package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@Autonomous(name = "Auto", group = "B")
public final class Auto extends LinearOpMode {
    public class Arm {
        private DcMotorEx shoulder;
        private DcMotorEx elbow;
        Servo clawClose;
        Servo clawSpin;
        Servo clawUp;
        public Arm(HardwareMap hardwareMap){
            shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
            //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

            elbow = hardwareMap.get(DcMotorEx.class, "elbow");
            //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);

            clawClose = hardwareMap.get(Servo.class, "clawClose");
            clawSpin = hardwareMap.get(Servo.class, "clawSpin");
            clawUp = hardwareMap.get(Servo.class, "clawUp");
        }
        double integralSumS = 0;
        double integralSumE = 0;
        double lastErrorS = 0;
        double lastErrorE = 0;
        ElapsedTime timerE = new ElapsedTime();
        ElapsedTime timerS = new ElapsedTime();
        double encoderPosition;
        double error;
        double derivative;
        double out;

        public double KpS = .002;
        public double KiS = .00000;
        public double KdS = 0.0001;
        public double maxPowUpS = 1;
        public double KpE = .003;
        public double KiE = .0000;
        public double KdE = 0.0001;
        public double maxPowUpE = .7;
        double shoulderAngle = 0;
        double elbowGravity = 0;
        double ticksPerRev = 1;
        double GravityK = .02;
        double GravityK2 = .15;
        public boolean shoulderTarget(double reference, double maxPowDownS) {
            // obtain the encoder position
            encoderPosition = -shoulder.getCurrentPosition();
            //telemetry.addData("shoulder pos: ", encoderPosition);

            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * GravityK + elbowGravity;

            out = (KpS * error) + (KiS * integralSumS) + (KdS * derivative);

            out = Math.max(Math.min(out, 1), -1);

            out += gravity;

//            if (out > maxPowUpS){
//                out = maxPowUpS;
//            } else if (out < - maxPowDownS){
//                out = -maxPowDownS;
//            }

            shoulder.setPower(out);

            lastErrorS = error;

            if (Math.abs(error) < 100) {
                shoulder.setPower(0);
                return false;
            } else {
                return true;
            }
        }
        public boolean elbowTarget( double reference, double maxPowDownE) {
            // obtain the encoder position
            encoderPosition = -elbow.getCurrentPosition();
            //telemetry.addData("elbow pos: ", encoderPosition);
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            double elbowAngle = ((encoderPosition + 3344)/ticksPerRev)*(2*Math.PI) + shoulderAngle;

            elbowGravity = Math.cos(elbowAngle) * GravityK2;

            out = (KpE * error) + (KiE * integralSumE) + (KdE * derivative) + elbowGravity;

            out = Math.max(Math.min(out, 1), -1);
//            if (out > maxPowUpE){
//                out = maxPowUpE;
//            } else if (out < - maxPowDownE){
//                out = -maxPowDownE;
//            }

            elbow.setPower(out);

            lastErrorE = error;

            // reset the timer for next time
            timerE.reset();

            if (Math.abs(error) < 50) {
                shoulder.setPower(0);
                return false;
            } else {
                return true;
            }
        }
        public class ShoulderMoveToHookReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return shoulderTarget(760, .2);
            }
        }
        public Action ShoulderMoveToHookReady() {
            return new ShoulderMoveToHookReady();
        }
        public class ShoulderMoveToHook implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return shoulderTarget(430/*430*/, .4);
            }
        }

        public Action ShoulderMoveToHook() {
            return new ShoulderMoveToHook();
        }

        public class ShoulderMovePickUpReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return shoulderTarget(2000, .4);
            }
        }

        public Action ShoulderMovePickUpReady() {
            return new ShoulderMovePickUpReady();
        }
        public class ShoulderMovePickUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return shoulderTarget(1450 /*1350*/, .4);
            }
        }

        public Action ShoulderMovePickUp() {
            return new ShoulderMovePickUp();
        }
        public class ShoulderMoveBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return shoulderTarget(3700/*3500*/, .4);
            }
        }

        public Action ShoulderMoveBasket() {
            return new ShoulderMoveBasket();
        }

        public class ElbowMoveToHookReady implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return elbowTarget(-781, .7);
            }
        }
        public Action ElbowMoveToHookReady() {
            return new ElbowMoveToHookReady();
        }
        public class ElbowMoveToPickUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return elbowTarget(-5500, .7);
            }
        }
        public Action ElbowMoveToPickUp() {
            return new ElbowMoveToPickUp();
        }
        public class ElbowMoveToDriving implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return elbowTarget(-400, .7);
            }
        }
        public Action ElbowMoveToDriving() {
            return new ElbowMoveToDriving();
        }
        public class ElbowMoveToBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return elbowTarget(-4000 /*4137*/, .7);
            }
        }
        public Action ElbowMoveToBasket() {
            return new ElbowMoveToBasket();
        }


        double startTime = 0;
        public class startClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.48);
                clawSpin.setPosition(.42);
                clawUp.setPosition(0);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action startClaw() {
            return new startClaw();
        }
        public class closeClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.48);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action CloseClaw() {
            return new closeClaw();
        }
        public class openClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.2);
                clawSpin.setPosition(.42);
                clawUp.setPosition(0);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action OpenClaw() {
            return new openClaw();
        }
        public class openClaw2 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.2);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action OpenClaw2() {
            return new openClaw2();
        }
        public class basketClaw2 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawUp.setPosition(.35);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action BasketClaw2() {
            return new basketClaw2();
        }
        public class reddyClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.25);
                clawSpin.setPosition(.42);
                clawUp.setPosition(.5);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action ReddyClaw() {
            return new reddyClaw();
        }
        public class basketClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawUp.setPosition(.2);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action basketClaw() {
            return new basketClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(6, 64, Math.PI*.75);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(6, 39), -Math.PI*.5)
        ;
        TrajectoryActionBuilder pathAwayFromRungs = drive.actionBuilder(new Pose2d(6, 39, -Math.PI*.25))
        //Action secondTrajectoryActionChosen = pathToBuckets.fresh()
                .setTangent(Math.PI*.5)
//                .splineTo(new Vector2d(48.5, 48.5), -Math.PI*.5);
                .splineToConstantHeading(new Vector2d(6, 46),0);
                //.splineToConstantHeading(new Vector2d(36, 24),-Math.PI*.5);
                //.build();
        TrajectoryActionBuilder pathTo1stSpikeMark = drive.actionBuilder(new Pose2d(6, 46, Math.PI*.75))
                .setTangent(0)
                //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);

                .splineToConstantHeading(new Vector2d(51, 52), 0);
        TrajectoryActionBuilder pathToPickUpBlock1 = drive.actionBuilder(new Pose2d(51, 52, Math.PI*.75))
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(51, 37), -Math.PI*.5);
        TrajectoryActionBuilder pathToBuckets = drive.actionBuilder(new Pose2d(49, 37, Math.PI*.75))
                .setTangent(Math.PI*.5)
                .splineToSplineHeading(new Pose2d(62, 68, Math.PI*.5), Math.PI*.25);
        TrajectoryActionBuilder pathTo2ndSpikeMark = drive.actionBuilder(new Pose2d(62, 68, Math.PI*.5))
                .setTangent(-Math.PI*.5)
                //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);
                //.splineToSplineHeading(new Pose2d(57, 52, Math.PI*.75), -Math.PI*.5)

                .splineToConstantHeading(new Vector2d(58, 52), -Math.PI*.5)
                .turnTo(Math.PI*.75);

        TrajectoryActionBuilder pathToPickUpBlock2 = drive.actionBuilder(new Pose2d(58, 52, Math.PI*.75))
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(58, 40), -Math.PI*.5);
        TrajectoryActionBuilder pathToBuckets2 = drive.actionBuilder(new Pose2d(58, 40, Math.PI*.75))
                .setTangent(Math.PI*.5)
                .splineToSplineHeading(new Pose2d(60, 71, Math.PI*.5), Math.PI*.25);



//        public void runOpMode() throws InterruptedException {
//            Pose2d initialPose = new Pose2d(6, 64, Math.PI*.75);
//
//            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//            Arm arm = new Arm(hardwareMap);
//
//            TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
//                    .setTangent(-Math.PI*.5)
//                    .splineToConstantHeading(new Vector2d(6, 40), -Math.PI*.5)
//                    ;
//            TrajectoryActionBuilder pathAwayFromRungs = drive.actionBuilder(new Pose2d(6, 40, -Math.PI*.25))
//                    //Action secondTrajectoryActionChosen = pathToBuckets.fresh()
//                    .setTangent(Math.PI*.5)
////                .splineTo(new Vector2d(48.5, 48.5), -Math.PI*.5);
//                    .splineToConstantHeading(new Vector2d(6, 46.5),0);
//            //.splineToConstantHeading(new Vector2d(36, 24),-Math.PI*.5);
//            //.build();
//            TrajectoryActionBuilder pathTo1stSpikeMark = drive.actionBuilder(new Pose2d(6, 46.5, Math.PI*.75))
//                    .setTangent(0)
//                    //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);
//
//                    .splineToConstantHeading(new Vector2d(52, 52), 0);
//            TrajectoryActionBuilder pathToPickUpBlock1 = drive.actionBuilder(new Pose2d(52, 52, Math.PI*.75))
//                    .setTangent(-Math.PI*.5)
//                    .splineToConstantHeading(new Vector2d(52, 37), -Math.PI*.5);
//            TrajectoryActionBuilder pathToBuckets = drive.actionBuilder(new Pose2d(42, 37, Math.PI*.75))
//                    .setTangent(Math.PI*.5)
//                    .splineToSplineHeading(new Pose2d(61, 72, Math.PI*.5), Math.PI*.25);
//            TrajectoryActionBuilder pathTo2ndSpikeMark = drive.actionBuilder(new Pose2d(61, 72, Math.PI*.5))
//                    .setTangent(-Math.PI*.5)
//                    //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);
//                    .splineToSplineHeading(new Pose2d(57, 52, Math.PI*.75), -Math.PI*.5);
//
//            TrajectoryActionBuilder pathToPickUpBlock2 = drive.actionBuilder(new Pose2d(57, 52, Math.PI*.75))
//                    .setTangent(-Math.PI*.5)
//                    .splineToConstantHeading(new Vector2d(57, 40), -Math.PI*.5);
//            TrajectoryActionBuilder pathToBuckets2 = drive.actionBuilder(new Pose2d(57, 40, Math.PI*.75))
//                    .setTangent(Math.PI*.5)
//                    .splineToSplineHeading(new Pose2d(61, 72, Math.PI*.5), Math.PI*.25);



        Action firstTrajectoryActionChosen = pathToRungs.build();
        Action secondTrajectoryActionChosen = pathAwayFromRungs.build();
        Action driveTo1stSpikeMark = pathTo1stSpikeMark.build();
        Action driveTo2ndSpikeMark = pathTo2ndSpikeMark.build();
        Action driveToPickUpBlock1 = pathToPickUpBlock1.build();
        Action driveToPickUpBlock2 = pathToPickUpBlock2.build();
        Action driveToBuckets = pathToBuckets.build();
        Action driveToBuckets2 = pathToBuckets2.build();

        telemetry.addData("velConstraint", drive.defaultVelConstraint);
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                    arm.startClaw(),
                    arm.ShoulderMoveToHookReady(),
                    firstTrajectoryActionChosen,
                    new ParallelAction(
                        arm.ShoulderMoveToHookReady(),
                        arm.ElbowMoveToHookReady()),
                    arm.ShoulderMoveToHook(),
                    arm.OpenClaw(),
                    arm.ElbowMoveToDriving(),
                    driveTo1stSpikeMark,
                    new ParallelAction(
                            arm.ShoulderMovePickUpReady(),
                            arm.ElbowMoveToPickUp(),
                            arm.ReddyClaw()),
                    arm.ShoulderMovePickUp(),
                    driveToPickUpBlock1,
                    arm.CloseClaw(),
                    driveToBuckets,
                    new ParallelAction(
                            arm.ShoulderMoveBasket(),
                            arm.ElbowMoveToBasket()),
                    //arm.basketClaw(),
                    arm.BasketClaw2(),
                    arm.OpenClaw2(),
                    new ParallelAction(
                            arm.ShoulderMovePickUpReady(),
                            arm.ElbowMoveToPickUp(),
                            arm.ReddyClaw()),
                    driveTo2ndSpikeMark,
                    arm.ShoulderMovePickUp(),
                    driveToPickUpBlock2,
                    arm.CloseClaw(),
                    driveToBuckets2,
                    new ParallelAction(
                            arm.ShoulderMoveBasket(),
                            arm.ElbowMoveToBasket()),
                    //arm.basketClaw(),
                    arm.BasketClaw2(),
                    arm.OpenClaw2()

//                    arm.MoveToReady(),

//                    new ParallelAction(
//                        driveToPickUpBlock1,
//                        intake.pickUpBlock(),
//                        arm.MoveToPickup()
//                    ),
//                    new ParallelAction(
//                            driveToBuckets,
//                            arm.MoveToHook()
//                    ),
//                    intake.spitBlockOut()

//                    new ParallelAction(
//                            arm.MoveToPickup(),
//                            secondTrajectoryActionChosen
//                    )
            ));
    }
}
