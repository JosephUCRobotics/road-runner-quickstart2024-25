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
@Autonomous(name = "Auto", group = "A")
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
        public double KpS = .01;
        public double KiS = .000001;
        public double KdS = 0.0001;
        public double maxPowUpS = .7;
        public double KpE = .01;
        public double KiE = .000001;
        public double KdE = 0.0001;
        public double maxPowUpE = .7;
        public boolean shoulderTarget(double reference, double maxPowDownS) {
            // obtain the encoder position
            encoderPosition = -shoulder.getCurrentPosition();
            telemetry.addData("shoulder pos: ", encoderPosition);
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            out = (KpS * error) + (KiS * integralSumS) + (KdS * derivative);
            if (out > maxPowUpS){
                out = maxPowUpS;
            } else if (out < - maxPowDownS){
                out = -maxPowDownS;
            }

            shoulder.setPower(out);

            lastErrorS = error;

            // reset the timer for next time
            timerS.reset();


            if (Math.abs(error) < 20) {
                shoulder.setPower(0);
                return false;
            } else {
                return true;
            }
        }
        public boolean elbowTarget( double reference, double maxPowDownE) {
            // obtain the encoder position
            encoderPosition = -elbow.getCurrentPosition();
            telemetry.addData("elbow pos: ", encoderPosition);
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            out = (KpE * error) + (KiE * integralSumE) + (KdE * derivative);
            if (out > maxPowUpE){
                out = maxPowUpE;
            } else if (out < - maxPowDownE){
                out = -maxPowDownE;
            }

            elbow.setPower(out);

            lastErrorE = error;

            // reset the timer for next time
            timerE.reset();

            if (Math.abs(error) < 20) {
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
                return shoulderTarget(430, .4);
            }
        }
        public Action ShoulderMoveToHook() {
            return new ShoulderMoveToHook();
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


        double startTime = 0;
        public class closeClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.45);
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
        public Action CloseClaw() {
            return new closeClaw();
        }
        public class openClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClose.setPosition(.2);
                clawSpin.setPosition(0.365);
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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(6, 64, Math.PI*.75);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(6, 38), -Math.PI*.5)
        ;
        TrajectoryActionBuilder pathAwayFromRungs = drive.actionBuilder(new Pose2d(6, 38, -Math.PI*.25))
        //Action secondTrajectoryActionChosen = pathToBuckets.fresh()
                .setTangent(Math.PI*.5)
//                .splineTo(new Vector2d(48.5, 48.5), -Math.PI*.5);
                .splineToConstantHeading(new Vector2d(6, 42),Math.PI*.5);
                //.splineToConstantHeading(new Vector2d(36, 24),-Math.PI*.5);
                //.build();
        TrajectoryActionBuilder pathTo1stSpikeMark = drive.actionBuilder(new Pose2d(6, 42, -Math.PI*.25))
                .setTangent(Math.PI*.5)
                //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);

                .splineTo(new Vector2d(27.5, 63.5), 0, new TranslationalVelConstraint(10))
                .splineTo(new Vector2d(35.5, 63.5), 0, new TranslationalVelConstraint(10))
                .splineTo(new Vector2d(47.5, 52), -Math.PI*.5, new TranslationalVelConstraint(10));
        TrajectoryActionBuilder pathToPickUpBlock1 = drive.actionBuilder(new Pose2d(47.5, 52, Math.PI*.75))
                .setTangent(-Math.PI*.5)
                .splineTo(new Vector2d(47.5, 35), -Math.PI*.5, new TranslationalVelConstraint(5));
        TrajectoryActionBuilder pathToBuckets = drive.actionBuilder(new Pose2d(47.5, 35, Math.PI*.75))
                .setTangent(Math.PI*.5)
                .splineToConstantHeading(new Vector2d(64.5, 52), Math.PI*.5);



        Action firstTrajectoryActionChosen = pathToRungs.build();
        Action secondTrajectoryActionChosen = pathAwayFromRungs.build();
        Action driveTo1stSpikeMark = pathTo1stSpikeMark.build();
        Action driveToPickUpBlock1 = pathToPickUpBlock1.build();
        Action driveToBuckets = pathToBuckets.build();
        telemetry.addData("velConstraint", drive.defaultVelConstraint);
        telemetry.update();
        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                    arm.CloseClaw(),
                    arm.ShoulderMoveToHookReady(),
                    firstTrajectoryActionChosen,
                    new ParallelAction(
                        arm.ShoulderMoveToHookReady(),
                        arm.ElbowMoveToHookReady()),
                    arm.ShoulderMoveToHook(),
                    arm.OpenClaw()
//                    intake.hookBlock(),
//                    arm.MoveToBasket(),
//                    new ParallelAction(
//                            firstTrajectoryActionChosen,
//                            intake.moveBlockToReadyPos()
//                    ),
//                    arm.MoveToHook(),
//                    secondTrajectoryActionChosen,
//                    intake.spitBlockOut(),
//                    arm.MoveToReady(),
//                    driveTo1stSpikeMark,
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
