package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;


public class MotorController {
//    public static double Acceleration = 500;
//    public static double MaxVelocity = 1000;
    ElapsedTime controllerTime = new ElapsedTime();

    Map<String, Double> consistences = new HashMap<>();

    Map<String, Double> moveG = new HashMap<>();

    boolean controllerOpen = true;
    double targetPosG = 0;

//    @Override
//    public void run() {
////        while (consistences.size() < 1){
////            try {
////                Thread.sleep(10);
////            } catch (InterruptedException e) {
////                throw new RuntimeException(e);
////            }
////        }
//        Map<String, Double> move = setMove(0,0, 1, 1);
//        // Code to be executed in the new thread
//
//
//        while (controllerOpen) {
//            targetPosG = calcCurrentTargetPos(move);
//            targetPosG = 1;
//            moveG.put("targetPos", targetPosG);
//        }
//    }
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

        moveG.put("P1", P1);
        moveG.put("P2",P2);
        moveG.put("P3",P3);
        moveG.put("length", length);
        moveG.put("moveEndTime", moveEndTime);
        moveG.put("moveStartTime", controllerTime.seconds());
        moveG.put("startPos", currentPos);
        moveG.put("accel", accel);
        moveG.put("peakVel", peakVel);
        moveG.put("targetPosG", targetPosG);

        // Set the start time of the move
        // This works because this function is only run ones at the beginning of each move
        return move;
    }

    public double calcCurrentTargetPos(Map<String, Double> move) {

        // Set the previous, current and total move time
        double moveTime = controllerTime.seconds() - move.get("moveStartTime");

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
    public Map<String, Double> getMoveData(){
        return moveG;
    }
    public void stopThread() {
        controllerOpen = false;
    }

    double lastErrorS = 0;
    double integralSumS = 0;
    public double calcPower(double currentPos, double targetPosG) {
        double ticksPerRev = 8192;
        // obtain the encoder position
        //telemetry.addData("shoulder pos: ", encoderPosition);

        // calculate the error
        double error = targetPosG - currentPos;

        // rate of change of the error
        double derivative = (error - lastErrorS) / controllerTime.seconds();

        // sum of all error over time
        integralSumS = integralSumS + (error * controllerTime.seconds());

        double shoulderAngle = ((currentPos - 1350)/ticksPerRev)*(2*Math.PI);

        double gravity = Math.cos(shoulderAngle) * consistences.get("gravityK");// + elbowGravity;

        double out = (consistences.get("Kp") * error) + (consistences.get("Ki") * integralSumS) + (consistences.get("Kd") * derivative);
//        double out = (consistences.get("Kp") * error) + (consistences.get("KdS") * derivative);
        out = Math.max(Math.min(out, 1), -1);

        out += gravity;

//            if (out > maxPowUpS){
//                out = maxPowUpS;
//            } else if (out < - maxPowDownS){
//                out = -maxPowDownS;
//            }


        lastErrorS = error;

        return out;
    }

    void setConsistences(double startPos, double Kp, double Ki, double Kd, double gravityK){
        consistences.put("startPos", startPos);
        consistences.put("Kp", Kp);
        consistences.put("Ki", Ki);
        consistences.put("Kd", Kd);
        consistences.put("gravityK", gravityK);

    }

}
