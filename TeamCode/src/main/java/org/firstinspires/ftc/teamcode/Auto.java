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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public final class Auto extends LinearOpMode {
    public class Arm {
        private DcMotorEx arm;
        public Arm(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        private boolean moveArmToPoss(double targetPos, double currentPos){
            double stopErr = 100;
            if (currentPos < targetPos-stopErr) {
                arm.setPower(1);
                return true;
            } else if (currentPos > targetPos+stopErr) {
                arm.setPower(-1);
                return true;
            } else {
                arm.setPower(0);
                return false;
            }
        }
        public class MoveToBasket implements Action {
            private double basketPos = 4525;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                return moveArmToPoss(basketPos, pos);
            }
        }
        public Action MoveToBasket() {
            return new MoveToBasket();
        }
        public class MoveToPickup implements Action {
            private double pickupPos = 12100;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                return moveArmToPoss(pickupPos, pos);
            }
        }
        public Action MoveToPickup() {
            return new MoveToPickup();
        }
        public class MoveToReady implements Action {
            private double readyPos = 11800;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                return moveArmToPoss(readyPos, pos);
            }
        }
        public Action MoveToReady() {
            return new MoveToReady();
        }
        public class MoveToHook implements Action {
            private double hookPos = 4188;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                return moveArmToPoss(hookPos, pos);
            }
        }
        public Action MoveToHook() {
            return new MoveToHook();
        }
    }


    public class Intake {
        private DcMotor claw;
        double startTime = 0;
        Servo hook;
        ColorSensor color;
        public Intake(HardwareMap hardwareMap){
            claw = hardwareMap.get(DcMotor.class, "claw");
            claw.setDirection(DcMotorSimple.Direction.REVERSE);

            hook = hardwareMap.get(Servo.class, "hook");

            color = hardwareMap.get(ColorSensor.class, "color");

        }
        public class spitBlockOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hook.setPosition(0);
                if (startTime == 0){
                    startTime = getRuntime();
                    claw.setPower(-1);
                    return true;
                } else if (getRuntime() > startTime + 1) {
                    claw.setPower(0);
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action spitBlockOut() {
            return new spitBlockOut();
        }
        public class pickUpBlock implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hook.setPosition(0);
                if (startTime == 0){
                    startTime = getRuntime();
                    claw.setPower(.8);
                    return true;
                } else if (getRuntime() > startTime + 10 || color.red() > 200) {
                    claw.setPower(0);
                    startTime = 0;
                    hook.setPosition(.1);
                    return false;
                } else return true;
            }
        }
        public Action pickUpBlock() {
            return new pickUpBlock();
        }
        public class hookBlock implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hook.setPosition(.1);
                return false;
            }
        }
        public Action hookBlock() {
            return new hookBlock();
        }

        public class moveBlockToReadyPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hook.setPosition(.08);
                if (startTime == 0){
                    startTime = getRuntime();
                    claw.setPower(-.5);
                    return true;
                } else if (getRuntime() > startTime + 1.5) {
                    claw.setPower(0);
                    startTime = 0;
                    hook.setPosition(.1);
                    return false;
                } else return true;
            }
        }
        public Action moveBlockToReadyPos() {
            return new moveBlockToReadyPos();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(6, 64, -Math.PI*.25);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(6, 34), -Math.PI*.5)
        ;
        TrajectoryActionBuilder pathAwayFromRungs = drive.actionBuilder(new Pose2d(6, 34, -Math.PI*.25))
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
                    intake.hookBlock(),
                    arm.MoveToBasket(),
                    new ParallelAction(
                            firstTrajectoryActionChosen,
                            intake.moveBlockToReadyPos()
                    ),
                    arm.MoveToHook(),
                    secondTrajectoryActionChosen,
                    intake.spitBlockOut(),
                    arm.MoveToReady(),
                    driveTo1stSpikeMark,
                    new ParallelAction(
                        driveToPickUpBlock1,
                        intake.pickUpBlock(),
                        arm.MoveToPickup()
                    ),
                    new ParallelAction(
                            driveToBuckets,
                            arm.MoveToHook()
                    ),
                    intake.spitBlockOut()

//                    new ParallelAction(
//                            arm.MoveToPickup(),
//                            secondTrajectoryActionChosen
//                    )
            ));
    }
}
