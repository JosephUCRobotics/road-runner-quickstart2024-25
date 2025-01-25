package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "AutoVelArm", group = "B")
public final class AutoVelArm extends LinearOpMode {
    public class Arm {
        private DcMotorEx shoulder;
        private DcMotorEx elbow;
        Servo clawClose;
        Servo clawSpin;
        Servo clawUp;
        public double KpS = 0.002;
        public double KiS = 0.000; //.002
        public double KdS = 0.0001;
        public double KpE = .003;
        public double KiE = .0000;
        public double KdE = 0.0001;
        public double GravityK = .02;
        public double GravityKE = 0.15;
        boolean armRunning = true;
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
        double lastErrorS = 0;
        ElapsedTime timerS = new ElapsedTime();
        int ticksPerRev = 8192;
        double elbowGravity = 0;
        double shoulderAngle = 0;

        public void shoulderTarget(double reference) {
            double maxPowDownS = 1;
            // obtain the encoder position
            double encoderPosition = -shoulder.getCurrentPosition();
            //telemetry.addData("shoulder pos: ", encoderPosition);

            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * GravityK + elbowGravity;

            double out = (KpS * error) + (KiS * integralSumS) + (KdS * derivative);

            out = Math.max(Math.min(out, 1), -1);

            out += gravity;

//            if (out > maxPowUpS){
//                out = maxPowUpS;
//            } else if (out < - maxPowDownS){
//                out = -maxPowDownS;
//            }

            shoulder.setPower(out);

            lastErrorS = error;

            // reset the timer for next time
            timerS.reset();
            telemetry.addData("Target Pos: ", reference/5000);
            telemetry.addData("Current Pos: ", encoderPosition/5000);
            telemetry.addData("P Pow: ", Math.max(Math.min(KpS * error, 1), -1));
            telemetry.addData("I Pow: ", Math.max(Math.min(KiS * integralSumS, 1), -1));
            telemetry.addData("D Pow: ", Math.max(Math.min(KdS * derivative, 1), -1));
            telemetry.addData("G Pow: ", gravity);
            telemetry.addData("out Pow: ",out);



//            if (Math.abs(error) < 20) {
//                moveArmToDropPos = false;
//                moveArmToHookPoss = false;
//                moveArmToPickupPos = false;
//                moveArmToReadyPos = false;
//            }
        }
        double integralSumE = 0;
        double lastErrorE = 0;
        ElapsedTime timerE = new ElapsedTime();
        public void elbowTarget( double reference) {
            // obtain the encoder position
            double encoderPosition = -elbow.getCurrentPosition();
            //telemetry.addData("elbow pos: ", encoderPosition);
            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            double elbowAngle = ((encoderPosition + 3344)/ticksPerRev)*(2*Math.PI) + shoulderAngle;

            elbowGravity = Math.cos(elbowAngle) * GravityKE;

            double out = (KpE * error) + (KiE * integralSumE) + (KdE * derivative) + elbowGravity;

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

            telemetry.addData("target", reference);
            telemetry.addData("pos", encoderPosition);
            //telemetry.addData("elbowAngle", elbowAngle);

//            if (Math.abs(error) < 20) {
//                moveArmToDropPos = false;
//                moveArmToHookPoss = false;
//                moveArmToPickupPos = false;
//                moveArmToReadyPos = false;
//            }
        }
//        public class ShoulderMoveToHookReady implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return shoulderTarget(760, .2);
//            }
//        }
//        public Action ShoulderMoveToHookReady() {
//            return new ShoulderMoveToHookReady();
//        }
String s = "";
        public class ShoulderMoveTo implements Action {
            double targetPos = 0;
            double Acceleration = 0;
            double MaxVelocity = 0;
            double currentPos;
            boolean shoulderMoveSet = false;

            public ShoulderMoveTo(double pos, double acc, double maxV){
                targetPos = pos;
                Acceleration = acc;
                MaxVelocity = maxV;
                //currentPos = -shoulder.getCurrentPosition();
                //move2 = setMove(currentPos, pos, acc, maxV);
                //s = pos + " " + acc + " " +maxV;
                s+=shoulder.getCurrentPosition()+" "+getRuntime();
                //telemetry.addData("runtime at first call", getRuntime());
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                telemetry.addData("runtime here " + move2.get("length").toString() + " " + move2.get("moveEndTime").toString(), getRuntime());
                telemetry.addData("s", s);
                //+ move.get("moveStartTime").toString() + " " + move.get("moveEndTime").toString()
                currentPos = -shoulder.getCurrentPosition();
                if (!shoulderMoveSet){
                    move2 = setMove(currentPos, targetPos, Acceleration, MaxVelocity);
                }
                shoulderMoveSet = true;
                if (getRuntime() > move2.get("moveStartTime") + move2.get("moveEndTime") /*&& Math.abs(shoulderTarget-currentPos) < 100*/) {
                    shoulderMoveSet = false;
                    return false;
                } else {return true;}
            }
        }

        public Action ShoulderMoveTo(double pos, double acc, double maxV) {
            telemetry.addData("runtime at action call", getRuntime());
            return new ShoulderMoveTo(pos, acc, maxV);
        }

        public class ElbowMoveTo implements Action {
            double targetPos = 0;
            double Acceleration = 0;
            double MaxVelocity = 0;
            double currentPos;
            boolean moveSet = false;

            public ElbowMoveTo(double pos, double acc, double maxV){
                targetPos = pos;
                Acceleration = acc;
                MaxVelocity = maxV;
                //currentPos = -shoulder.getCurrentPosition();
                //move2 = setMove(currentPos, pos, acc, maxV);
                //s = pos + " " + acc + " " +maxV;
                //telemetry.addData("runtime at first call", getRuntime());
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //+ move.get("moveStartTime").toString() + " " + move.get("moveEndTime").toString()
                currentPos = -elbow.getCurrentPosition();
                if (!moveSet){
                    moveE = setMove(currentPos, targetPos, Acceleration, MaxVelocity);
                }
                moveSet = true;
                if (getRuntime() > moveE.get("moveStartTime") + moveE.get("moveEndTime") /*&& Math.abs(elbowTarget-currentPos) < 100*/) {
                    moveSet = false;
                    return false;
                } else {return true;}
            }
        }

        public Action ElbowMoveTo(double pos, double acc, double maxV) {
            return new ElbowMoveTo(pos, acc, maxV);
        }

//        public class ShoulderMovePickUpReady implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return shoulderTarget(2000, .4);
//            }
//        }
//
//        public Action ShoulderMovePickUpReady() {
//            return new ShoulderMovePickUpReady();
//        }
//        public class ShoulderMovePickUp implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return shoulderTarget(1350 /*1450*/, .4);
//            }
//        }
//
//        public Action ShoulderMovePickUp() {
//            return new ShoulderMovePickUp();
//        }
//        public class ShoulderMoveBasket implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return shoulderTarget(3700/*3500*/, .4);
//            }
//        }
//
//        public Action ShoulderMoveBasket() {
//            return new ShoulderMoveBasket();
//        }
//
//        public class ElbowMoveToHookReady implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return elbowTarget(-781, .7);
//            }
//        }
//        public Action ElbowMoveToHookReady() {
//            return new ElbowMoveToHookReady();
//        }
//        public class ElbowMoveToPickUp implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return elbowTarget(-5500, .7);
//            }
//        }
//        public Action ElbowMoveToPickUp() {
//            return new ElbowMoveToPickUp();
//        }
//        public class ElbowMoveToDriving implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return elbowTarget(-400, .7);
//            }
//        }
//        public Action ElbowMoveToDriving() {
//            return new ElbowMoveToDriving();
//        }
//        public class ElbowMoveToBasket implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                return elbowTarget(-4000 /*4137*/, .7);
//            }
//        }
//        public Action ElbowMoveToBasket() {
//            return new ElbowMoveToBasket();
//        }


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
        double shoulderTarget = 0;
        double elbowTarget = 0;
        public class upDateArmLoop implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shoulderTarget = calcCurrentTargetPos(move2);

                elbowTarget = calcCurrentTargetPos(moveE);

                shoulderTarget(shoulderTarget);
                elbowTarget(elbowTarget);

                telemetry.addData("shoulder Target", shoulderTarget);
                telemetry.update();

                if (armRunning){return true;} else{return false;}
            }
        }
        public Action upDateArm() {
            return new upDateArmLoop();
        }
        Map<String, Double> move2;
        Map<String, Double> moveE;

        public void setInitMoves() {
            double currentPos = -shoulder.getCurrentPosition();
            double elbowCurrentPos = -elbow.getCurrentPosition();

            move2 = setMove(currentPos, currentPos, 1, 1);
            moveE = setMove(elbowCurrentPos, elbowCurrentPos, 1, 1);

        }

        public Map<String, Double> setMove(double currentPos, double targetPos, double accel, double peakVel) {

            //s += "move is set";

            Map<String, Double> move = new HashMap<>();

            // Set the total distance of the move
            double length = targetPos - currentPos;

            accel = Math.abs(accel);
            peakVel = Math.abs(peakVel);

            //if (length < 0){
            //    accel *= -1;
            //    peakVel *= -1;
            //}

            // Set the pos at T1 = 1/2 Vmax * t1
            // If the triangle was the biggest size is just length / 2
            double P1 = -0.5 * (Math.pow(peakVel, 2) / accel);

            double moveEndTime;

            // Decide if the move is a trapezoid or a triangle
            if (Math.abs(length) <= 2 * -P1) {
                // Triangle
                // Calc the move end time 2 + Vmax/A ????????
                moveEndTime = 2 * Math.sqrt(Math.abs(length) / accel);

            } else {
                // Trapezoid
                // Calc the move end time L/Vmax + Vmax/A
                moveEndTime = Math.abs(length) / peakVel + peakVel / accel;
            }
            double P2 = accel * moveEndTime;
            double P3 = Math.abs(length) - 0.5 * accel * Math.pow(moveEndTime, 2);

            if (length < 0){
                accel *= -1;
                peakVel *= -1;
                P1 *= -1;
                P2 *= -1;
                P3 *= -1;
            }

            move.put("P1", P1);
            move.put("P2",P2);
            move.put("P3",P3);
            move.put("length", length);
            move.put("moveEndTime", moveEndTime);
            move.put("moveStartTime",  getRuntime());
            move.put("startPos", currentPos);
            move.put("accel", accel);
            move.put("peakVel", peakVel);
            //s += getRuntime();
            // Set the start time of the move
            // This works because this function is only run ones at the beginning of each move
            return move;
        }
        public double calcCurrentTargetPos(Map<String, Double> move) {

            // Set the previous, current and total move time
            double moveTime = getRuntime() - move.get("moveStartTime");

            double moveEndTime = move.get("moveEndTime");
            double accel = move.get("accel");
            double P1 = move.get("P1");
            double P2 = move.get("P2");
            double P3 = move.get("P3");
            double peakVel = move.get("peakVel");
            telemetry.addData("P1", P1);
            // if the move is over set the move to go to the last pos by setting the time to the last time
            if (moveTime > moveEndTime) {
                moveTime = moveEndTime;
            }

            double Pmove;
            if (Math.abs(move.get("length")) <= 2 * Math.abs(P1)) {
                // Triangle
                if (moveTime <= moveEndTime / 2) {
                    Pmove =  0.5 * accel * Math.pow(moveTime, 2);
                } else {
                    Pmove = -0.5 * accel * Math.pow(moveTime, 2) + P2 * moveTime + P3;
                }
            } else {
                // Trapezoid
                double timeA = peakVel / accel;
                if (moveTime <= timeA) {
                    Pmove = 0.5 * accel * Math.pow(moveTime, 2);
                } else if (moveTime <= moveEndTime - timeA) {
                    Pmove = peakVel * moveTime + P1;
                } else {
                    Pmove = -0.5 * accel * Math.pow(moveTime, 2) + P2 * moveTime + P3;
                }
            }
            double targetPos = move.get("startPos") + Pmove;
            return targetPos;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(6, 64, Math.PI*.75);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(6, 41), -Math.PI*.5)
                ;
        TrajectoryActionBuilder pathAwayFromRungs = drive.actionBuilder(new Pose2d(6, 41, -Math.PI*.25))
                //Action secondTrajectoryActionChosen = pathToBuckets.fresh()
                .setTangent(Math.PI*.5)
//                .splineTo(new Vector2d(48.5, 48.5), -Math.PI*.5);
                .splineToConstantHeading(new Vector2d(6, 46),0);
        //.splineToConstantHeading(new Vector2d(36, 24),-Math.PI*.5);
        //.build();
        TrajectoryActionBuilder pathTo1stSpikeMark = drive.actionBuilder(new Pose2d(6, 46, Math.PI*.75))
                .setTangent(Math.PI*.5)
                //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);

                .splineToConstantHeading(new Vector2d(51, 52), 0);
        TrajectoryActionBuilder pathToPickUpBlock1 = drive.actionBuilder(new Pose2d(51, 52, Math.PI*.75))
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(51, 41), -Math.PI*.5);
        TrajectoryActionBuilder pathToBuckets = drive.actionBuilder(new Pose2d(49, 41, Math.PI*.75))
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

        TrajectoryActionBuilder pathTo3rdSpikeMark = drive.actionBuilder(new Pose2d(60, 71, Math.PI*.5))
                .setTangent(-Math.PI*.5)
                //.splineToSplineHeading(new Pose2d(46, 46, Math.PI*.75),-Math.PI*.5);
                //.splineToSplineHeading(new Pose2d(57, 52, Math.PI*.75), -Math.PI*.5)

                .splineToConstantHeading(new Vector2d(58, 52), -Math.PI*.5)
                .turnTo(Math.PI*.75);

        TrajectoryActionBuilder pathToPickUpBlock3 = drive.actionBuilder(new Pose2d(58, 52, Math.PI*.75))
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(58, 40), -Math.PI*.5);
        TrajectoryActionBuilder pathToBuckets3 = drive.actionBuilder(new Pose2d(58, 40, Math.PI*.75))
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
        Action driveTo3rdSpikeMark = pathTo3rdSpikeMark.build();
        Action driveToPickUpBlock1 = pathToPickUpBlock1.build();
        Action driveToPickUpBlock2 = pathToPickUpBlock2.build();
        Action driveToPickUpBlock3 = pathToPickUpBlock3.build();
        Action driveToBuckets = pathToBuckets.build();
        Action driveToBuckets2 = pathToBuckets2.build();
        Action driveToBuckets3 = pathToBuckets3.build();

        telemetry.addData("velConstraint", drive.defaultVelConstraint);
        telemetry.update();

        arm.setInitMoves();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        arm.upDateArm(),
                        new SequentialAction(
                                arm.startClaw(),
                                //arm.ShoulderMoveTo(760, 4000, 2000),
                                new ParallelAction(
                                        arm.ShoulderMoveTo(760, 4000, 2000),
                                        firstTrajectoryActionChosen,
                                        arm.ElbowMoveTo(-781, 2000, 2000)),
                                arm.ShoulderMoveTo(300, 4000, 4000),
                                arm.OpenClaw(),
//                                arm.ElbowMoveTo(-400, 4000, 4000),
//                                driveTo1stSpikeMark,
                                new ParallelAction(
//                                        arm.ShoulderMoveTo(1350, 2000, 2000),
                                        driveTo1stSpikeMark,
//                                        arm.ElbowMoveTo(-5500, 3000, 3000),
                                        arm.ReddyClaw())
//                                driveToPickUpBlock1,
//                                arm.CloseClaw()
//                                new ParallelAction(
//                                        driveToBuckets,
//                                        arm.ShoulderMoveTo(3700, 500, 1500),
//                                        arm.ElbowMoveTo(-4000, 500, 1500)),
//                                //arm.basketClaw(),
//                                arm.BasketClaw2(),
//                                arm.OpenClaw2(),
//                                new ParallelAction(
//                                        arm.ShoulderMoveTo(1350, 500, 1000),
//                                        arm.ElbowMoveTo(-5500, 1000, 1000),
//                                        driveTo2ndSpikeMark,
//                                        arm.ReddyClaw()),
//                                driveToPickUpBlock2,
//                                arm.CloseClaw(),
//                                new ParallelAction(
//                                        driveToBuckets2,
//                                        arm.ShoulderMoveTo(3700, 500, 1500),
//                                        arm.ElbowMoveTo(-4000, 500, 1500)),
//                                //arm.basketClaw(),
//                                arm.BasketClaw2(),
//                                arm.OpenClaw2(),
//                                new ParallelAction(
//                                        arm.ShoulderMoveTo(1350, 500, 1000),
//                                        arm.ElbowMoveTo(-5500, 1000, 1000),
//                                        driveTo3rdSpikeMark,
//                                        arm.ReddyClaw()),
//                                driveToPickUpBlock3,
//                                arm.CloseClaw(),
//                                new ParallelAction(
//                                        driveToBuckets3,
//                                        arm.ShoulderMoveTo(3700, 500, 1500),
//                                        arm.ElbowMoveTo(-4000, 500, 1500)),
//                                //arm.basketClaw(),
//                                arm.BasketClaw2(),
//                                arm.OpenClaw2()
                        )
                )
        );
    }
}