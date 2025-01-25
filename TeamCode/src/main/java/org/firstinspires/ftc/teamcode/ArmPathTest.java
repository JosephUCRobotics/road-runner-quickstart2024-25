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

@TeleOp(name = "Arm Path Test", group = "B")
@Config
public class ArmPathTest extends LinearOpMode {
    private DcMotorEx shoulder;
    private DcMotorEx elbow;

    public static double Acceleration = 500;
    public static double MaxVelocity = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        double currentPos = shoulder.getCurrentPosition();

        Map<String, Double> move = setMove(currentPos, currentPos, Acceleration, MaxVelocity);

        while (opModeIsActive()) {
            double shoulderTarget = calcCurrentTargetPos(move);
            telemetry.addData("shoulderTarget", shoulderTarget);
            telemetry.addData("startPos", move.get("startPos"));
            telemetry.addData("P1", move.get("P1"));
            telemetry.addData("P2", move.get("P2"));
            telemetry.addData("P3", move.get("P3"));
            telemetry.addData("length", move.get("length"));
            telemetry.addData("moveStartTime", move.get("moveStartTime"));
            telemetry.addData("moveEndTime", move.get("moveEndTime"));
            telemetry.addData("runtime", getRuntime());


            if (gamepad1.y) {
                move = setMove(shoulderTarget, 3000, Acceleration, MaxVelocity);
            } else if (gamepad1.a) {
                move = setMove(shoulderTarget, 1000, Acceleration, MaxVelocity);
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
}
