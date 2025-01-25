package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Arm Path With Motor Control", group = "B")
@Config
public class ArmPathWithMotorControl extends LinearOpMode {
    private DcMotorEx shoulder;
    private DcMotorEx elbow;

    public static double Acceleration = 500;
    public static double MaxVelocity = 1000;
    public static double AccelerationE = 500;
    public static double MaxVelocityE = 1000;
    public static double KpS = 0.002;
    public static double KiS = 0.000; //.002
    public static double KdS = 0.0001;
    public static double KpE = .003;
    public static double KiE = .0000;
    public static double KdE = 0.0001;
    public static double GravityK = .02;
    public static double GravityKE = 0.15;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        double currentPos = -shoulder.getCurrentPosition();
        double elbowCurrentPos = -elbow.getCurrentPosition();

        Map<String, Double> move = setMove(currentPos, currentPos, Acceleration, MaxVelocity);
        Map<String, Double> moveE = setMove(elbowCurrentPos, elbowCurrentPos, Acceleration, MaxVelocity);

        while (opModeIsActive()) {
            double shoulderTarget = calcCurrentTargetPos(move);
            double elbowTarget = calcCurrentTargetPos(moveE);
            telemetry.addData("shoulderTarget", shoulderTarget);
            telemetry.addData("startPos", move.get("startPos"));
            telemetry.addData("P1", move.get("P1"));
            telemetry.addData("P2", move.get("P2"));
            telemetry.addData("P3", move.get("P3"));
            telemetry.addData("length", move.get("length"));
            telemetry.addData("moveStartTime", move.get("moveStartTime"));
            telemetry.addData("moveEndTime", move.get("moveEndTime"));
            telemetry.addData("runtime", getRuntime());
            shoulder.setPower(shoulderTarget(shoulderTarget));
            elbowTarget(elbowTarget);
            //shoulderTarget(shoulderTarget);


            if (gamepad1.y) {
                // basket pos
                move = setMove(-shoulder.getCurrentPosition(), 3700, Acceleration, MaxVelocity);
                moveE = setMove(-elbow.getCurrentPosition(), -4000, AccelerationE, MaxVelocityE);
                integralSumS = 0;
                integralSumE = 0;
            } else if (gamepad1.a) {
                // start pos
                move = setMove(-shoulder.getCurrentPosition(), 0, Acceleration, MaxVelocity);
                moveE = setMove(-elbow.getCurrentPosition(), -0, AccelerationE, MaxVelocityE);
                integralSumS = 0;
                integralSumE = 0;
            } else if (gamepad1.x) {
                // pick up block
                move = setMove(-shoulder.getCurrentPosition(), 1350, Acceleration, MaxVelocity);
                moveE = setMove(-elbow.getCurrentPosition(), -5500, AccelerationE, MaxVelocityE);
                integralSumS = 0;
                integralSumE = 0;
            } else if (gamepad1.b) {
                //hook Block
                move = setMove(-shoulder.getCurrentPosition(), 430, Acceleration, MaxVelocity);
                moveE = setMove(-elbow.getCurrentPosition(), -781, AccelerationE, MaxVelocityE);
                integralSumS = 0;
                integralSumE = 0;
            }
            telemetry.update();
        }
    }
    public Map<String, Double> setMove(double currentPos, double targetPos, double accel, double peakVel) {
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
        move.put("moveStartTime", getRuntime());
        move.put("startPos", currentPos);
        move.put("accel", accel);
        move.put("peakVel", peakVel);

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
    double integralSumS = 0;
    double lastErrorS = 0;
    ElapsedTime timerS = new ElapsedTime();
    int ticksPerRev = 8192;
    double elbowGravity = 0;
    double shoulderAngle = 0;

    public double shoulderTarget(double reference) {
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

        //shoulder.setPower(out);

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
        return out;
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
}
