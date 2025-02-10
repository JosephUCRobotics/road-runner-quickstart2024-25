package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "AutoRungSecondBlockQualV2", group = "B")
public final class AutoRungSecondBlockV2 extends LinearOpMode {
    double initialOffset = 0;
    public class Arm {
        private DcMotorEx shoulder;
        private DcMotorEx shoulderBreak;
        private DcMotorEx elbow;
        Servo clawClose;
        CRServo clawWheels;
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

        Map<String, Double> firstSMoveToBuckets;
        Map<String, Double> secondSMoveToBuckets;
        Map<String, Double> firstSMoveToGround;
        Map<String, Double> firstEMoveToBuckets;
        Map<String, Double> secondEMoveToBuckets;
        Map<String, Double> firstEMoveToGround;
        public Arm(HardwareMap hardwareMap){
            shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
            //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

            shoulderBreak = hardwareMap.get(DcMotorEx.class, "brake");
            //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulderBreak.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulderBreak.setDirection(DcMotorSimple.Direction.REVERSE);


            elbow = hardwareMap.get(DcMotorEx.class, "elbow");
            //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);

            clawClose = hardwareMap.get(Servo.class, "clawClose");
            clawWheels = hardwareMap.get(CRServo.class, "clawWheels");
            clawUp = hardwareMap.get(Servo.class, "clawUp");

            firstSMoveToBuckets = setMove(0, 3700, 1500, 2000);
            firstSMoveToGround = setMove(3700, 1350, 4000, 4000);
            secondSMoveToBuckets = setMove(1350, 3700, 3000, 3000);

            firstEMoveToBuckets = setMove(0, -4000, 1500, 2000);
            firstEMoveToGround = setMove(-4000, -5500, 1500, 2000);
            secondEMoveToBuckets = setMove(-5500, -4000, 3000, 3000);
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

            public ShoulderMoveTo(double pos, double acc, double maxV, double curPos){
                targetPos = pos;
                Acceleration = acc;
                MaxVelocity = maxV;
                currentPos = curPos;
                //currentPos = -shoulder.getCurrentPosition();
                //move2 = setMove(currentPos, pos, acc, maxV);
                //s = pos + " " + acc + " " +maxV;
                //s+=shoulder.getCurrentPosition()+" "+getRuntime();
                //telemetry.addData("runtime at first call", getRuntime());
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                telemetry.addData("runtime here " + move2.get("length").toString() + " " + move2.get("moveEndTime").toString(), getRuntime());
                telemetry.addData("s", s);
                //+ move.get("moveStartTime").toString() + " " + move.get("moveEndTime").toString()

                if (!shoulderMoveSet){
                    if (targetPos == 3700 && currentPos == 0){
                        move2 = firstSMoveToBuckets;
                    } else if (targetPos == 1350 && currentPos == 3700) {
                        move2 = firstSMoveToGround;
                    } else if (targetPos == 3700 && currentPos == 1350) {
                        move2 = secondSMoveToBuckets;
                    } else {
                        move2 = setMove(currentPos, targetPos, Acceleration, MaxVelocity);
                    }
                    move2.put("moveStartTime",  getRuntime());
                }

                currentPos = -shoulder.getCurrentPosition();

                shoulderMoveSet = true;
                if (getRuntime() > move2.get("moveStartTime") + move2.get("moveEndTime") && Math.abs(shoulderTarget-currentPos) < 200) {
                    shoulderMoveSet = false;
                    return false;
                } else {return true;}
            }
        }

        public Action ShoulderMoveTo(double pos, double acc, double maxV) {
            telemetry.addData("runtime at action call", getRuntime());
            return new ShoulderMoveTo(pos, acc, maxV, -shoulder.getCurrentPosition());
        }
        public Action ShoulderMoveTo(double pos, double acc, double maxV, double curPos) {
            telemetry.addData("runtime at action call", getRuntime());
            return new ShoulderMoveTo(pos, acc, maxV,curPos);
        }

        public class ElbowMoveTo implements Action {
            double targetPos = 0;
            double Acceleration = 0;
            double MaxVelocity = 0;
            double currentPos;
            boolean moveSet = false;

            public ElbowMoveTo(double pos, double acc, double maxV, double curPos){
                targetPos = pos;
                Acceleration = acc;
                MaxVelocity = maxV;
                currentPos = curPos;
                //currentPos = -shoulder.getCurrentPosition();
                //move2 = setMove(currentPos, pos, acc, maxV);
                //s = pos + " " + acc + " " +maxV;
                //telemetry.addData("runtime at first call", getRuntime());
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //+ move.get("moveStartTime").toString() + " " + move.get("moveEndTime").toString()

                if (!moveSet){
                    if (targetPos == -4000 && currentPos == 0) {
                        s = "in if";
                        moveE = firstEMoveToBuckets;
                    } else if (targetPos == -5500 && currentPos == -4000) {
                        s = "in if";
                        moveE = firstEMoveToGround;
                    } else if (targetPos == -4000 && currentPos == -5500) {
                        s = "in if";
                        moveE = secondEMoveToBuckets;
                    } else {
                        s = "not in if";
                        moveE = setMove(currentPos, targetPos, Acceleration, MaxVelocity);
                    }
                    moveE.put("moveStartTime",  getRuntime());
                }

                currentPos = -elbow.getCurrentPosition();

                moveSet = true;
                if (getRuntime() > moveE.get("moveStartTime") + moveE.get("moveEndTime") && Math.abs(elbowTarget-currentPos) < 200) {
                    moveSet = false;
                    return false;
                } else {return true;}
            }
        }

        public Action ElbowMoveTo(double pos, double acc, double maxV) {
            return new ElbowMoveTo(pos, acc, maxV, -elbow.getCurrentPosition());
        }
        public Action ElbowMoveTo(double pos, double acc, double maxV, double curPos) {
            return new ElbowMoveTo(pos, acc, maxV, curPos);
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

        public class pullBackClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawUp.setPosition(.25);
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action pullBackClaw() {
            return new pullBackClaw();
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
                clawClose.setPosition(.35); // .2
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
                clawUp.setPosition(.4); // .5
                if (startTime == 0){
                    startTime = getRuntime();
                    return true;
                } else if (getRuntime() > startTime + .5) {
                    startTime = 0;
                    return false;
                } else return true;
            }
        }
        public Action reddyClaw() {
            return new reddyClaw();
        }

        public class flatClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
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
        public Action FlatClaw() {
            return new flatClaw();
        }
        public class basketClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawUp.setPosition(.2); // .1
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
        double minBreakPow = .15;
        double breakMultiplyer = 1.75;
        double breakMultiplyerDown = .15;
        public class upDateArmLoop implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Map<String, Double> shoulderPosAndVel = calcCurrentTargetPos(move2);
                double shoulderTarget = shoulderPosAndVel.get("targetPos");
                double shoulderTargetVel = shoulderPosAndVel.get("targetVel");

                double breakPow = 0;
                if (shoulderTargetVel > 0) {
                    breakPow = (shoulderTargetVel/3000) * (1-minBreakPow)*breakMultiplyer + minBreakPow;
                } else {
                    breakPow = (shoulderTargetVel/3000) * (1-minBreakPow)*breakMultiplyerDown + minBreakPow;
                }

                shoulderBreak.setPower(breakPow);

                Map<String, Double> elbowPosAndVel = calcCurrentTargetPos(moveE);
                double elbowTarget = elbowPosAndVel.get("targetPos");

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
        public Map<String, Double> calcCurrentTargetPos(Map<String, Double> move) {

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
            double Vmove;
            if (Math.abs(move.get("length")) <= 2 * Math.abs(P1)) {
                // Triangle
                if (moveTime <= moveEndTime / 2) {
                    Pmove =  0.5 * accel * Math.pow(moveTime, 2);
                    Vmove = accel * moveTime;
                } else {
                    Pmove = -0.5 * accel * Math.pow(moveTime, 2) + P2 * moveTime + P3;
                    Vmove = -accel * moveTime + P2;
                }
            } else {
                // Trapezoid
                double timeA = peakVel / accel;
                if (moveTime <= timeA) {
                    Pmove = 0.5 * accel * Math.pow(moveTime, 2);
                    Vmove = accel * moveTime;
                } else if (moveTime <= moveEndTime - timeA) {
                    Pmove = peakVel * moveTime + P1;
                    Vmove = peakVel;
                } else {
                    Pmove = -0.5 * accel * Math.pow(moveTime, 2) + P2 * moveTime + P3;
                    Vmove = -accel * moveTime + P2;
                }
            }
            double targetPos = move.get("startPos") + Pmove;
            Map<String, Double> returnThis = new HashMap<>();
            returnThis.put("targetPos", targetPos);
            returnThis.put("targetVel", Vmove);
            return returnThis;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        while (!gamepad1.a){
            if (gamepad1.dpad_up){
                initialOffset += .25;
            } else if (gamepad1.dpad_down) {
                initialOffset -= .25;
            }
            telemetry.addData("Initial Offset", initialOffset);
            telemetry.update();
            sleep(250);
        }


        Pose2d initialPose = new Pose2d(-6+initialOffset, 64, Math.PI*.75);
        //Pose2d initialPose = new Pose2d(62, 60, Math.PI*.5);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder pathToRungs = drive.actionBuilder(initialPose)
                .setTangent(-Math.PI*.5)
                .splineToConstantHeading(new Vector2d(-6, 40), -Math.PI*.5, null, new ProfileAccelConstraint(-15, 15));

        TrajectoryActionBuilder pathTo1stSpikeMark = drive.actionBuilder(new Pose2d(-6, 40, Math.PI*.75))
                .setTangent(Math.PI*.7)
                .splineToSplineHeading(new Pose2d(-28, 50, Math.PI*.75), Math.PI, null, new ProfileAccelConstraint(-15, 15));

        TrajectoryActionBuilder pathToPickUpBlock1 = drive.actionBuilder(new Pose2d(-29, 50, Math.PI*.75))
                .turnTo(Math.PI*.5);

        TrajectoryActionBuilder PathToHuman = drive.actionBuilder(new Pose2d(-29, 50, Math.PI*.5))
                .turnTo(0);

        TrajectoryActionBuilder PathToWate = drive.actionBuilder(new Pose2d(-29, 50, 0))
                .turnTo(Math.PI*.25)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-5, 50, Math.PI*.25), 0, null, new ProfileAccelConstraint(-15, 15))
                .setTangent(Math.PI*.5)
                .splineToSplineHeading(new Pose2d(-5, 62, Math.PI*.25), Math.PI * .5, null, new ProfileAccelConstraint(-15, 15));

        TrajectoryActionBuilder pathTo2ndSpikeMark = drive.actionBuilder(new Pose2d(-28, 50, Math.PI*.1))
                .turnTo(Math.PI*.5);

        TrajectoryActionBuilder pathToPickUpBlock2 = drive.actionBuilder(new Pose2d(-28, 50, Math.PI*.5))
                .setTangent(Math.PI*-.8)
                .splineToConstantHeading(new Vector2d(-48, 44), Math.PI*-.8, null, new ProfileAccelConstraint(-15, 15));

        TrajectoryActionBuilder SecondPathToHuman = drive.actionBuilder(new Pose2d(-48, 44, Math.PI*.5))
                .setTangent(Math.PI*.25)
                .splineToSplineHeading(new Pose2d(-36, 62, Math.PI*.05), Math.PI*-.5, null, new ProfileAccelConstraint(-15, 15));

        TrajectoryActionBuilder PathToHook1 = drive.actionBuilder(new Pose2d(-36, 62, Math.PI*.05))
                .turnTo(Math.PI*.25);


        TrajectoryActionBuilder PathToPickUpHook1 = drive.actionBuilder(new Pose2d(-5, 62, Math.PI*.25))
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-27, 62), -Math.PI);
        TrajectoryActionBuilder PathToRung = drive.actionBuilder(new Pose2d(-27, 62, Math.PI*.25))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(0, 62), 0);
        TrajectoryActionBuilder PathToRung2 = drive.actionBuilder(new Pose2d(0, 62, Math.PI*.25))
                .turnTo(Math.PI*.75);
        TrajectoryActionBuilder PathToRung3 = drive.actionBuilder(new Pose2d(0, 62, Math.PI*75))
                .setTangent(Math.PI * -.5)
                .splineToConstantHeading(new Vector2d(0, 48), Math.PI*-.5);



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
        Action driveTo1stSpikeMark = pathTo1stSpikeMark.build();
        Action driveTo2ndSpikeMark = pathTo2ndSpikeMark.build();
        Action pathToHook1 = PathToHook1.build();
        Action driveToPickUpBlock1 = pathToPickUpBlock1.build();
        Action driveToPickUpBlock2 = pathToPickUpBlock2.build();
        Action driveToPickUpHook1 = PathToPickUpHook1.build();
        Action firstPathToHuman = PathToHuman.build();
        Action secondPathToHuman = SecondPathToHuman.build();
        Action driveToRung = PathToRung.build();
        Action driveToRung2 = PathToRung2.build();
        Action driveToRung3 = PathToRung3.build();
        Action driveToWate = PathToWate.build();

        telemetry.addData("velConstraint", drive.defaultVelConstraint);
        telemetry.update();

        arm.setInitMoves();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        arm.upDateArm(),
                        new SequentialAction(
                                new ParallelAction(
                                        arm.startClaw(),
                                        firstTrajectoryActionChosen,
                                        arm.ShoulderMoveTo(760, 1500, 2000, 0),
                                        arm.ElbowMoveTo(-780, 1500, 2000, 0)),
                                arm.ShoulderMoveTo(350, 1500, 2000, 760),
                                arm.OpenClaw2(),
                                driveTo1stSpikeMark,
                                new ParallelAction(
                                        arm.OpenClaw(),
                                        arm.ElbowMoveTo(-2650, 1500, 2000, -780),
                                        driveToPickUpBlock1),
                                arm.CloseClaw(),
                                new ParallelAction(
                                        firstPathToHuman,
                                        arm.ElbowMoveTo(-5695, 1500, 2000, -2650),
                                        arm.ShoulderMoveTo(1600, 1500, 2000, 350),
                                        arm.FlatClaw()),
                                arm.OpenClaw(),
                                driveToWate,

                                // Leave disabled
//                                new ParallelAction(
//                                        arm.ShoulderMoveTo(1340, 1500, 2000, 1600),
//                                        driveToPickUpBlock2),
//                                arm.CloseClaw(),
//                                secondPathToHuman,
//                                arm.ShoulderMoveTo(1700, 1500, 2000, 1340),
//                                arm.OpenClaw(),
//                                pathToHook1,


                                arm.ShoulderMoveTo(1340, 1500, 2000, 1600),

                                //new SleepAction(1),
                                driveToPickUpHook1,
                                arm.CloseClaw()
//                                driveToRung,
//                                new ParallelAction(
//                                        driveToRung2,
//                                        arm.ShoulderMoveTo(3000, 1500, 2000, 1340)
//                                ),
//                                driveToRung3,
//                                arm.FlatClaw(),
//                                arm.ElbowMoveTo(-6000, 1500, 2000, -5695),
//                                arm.OpenClaw()
                        )
                )
        );
    }
}