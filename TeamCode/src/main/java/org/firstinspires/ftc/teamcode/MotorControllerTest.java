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

@TeleOp(name = "Motor Controller Test", group = "B")
@Config
public class MotorControllerTest extends LinearOpMode {
    private DcMotorEx shoulder;
    private DcMotorEx elbow;

    public static double Acceleration = 100;
    public static double MaxVelocity = 1000;
    public static double KpS = 0.002;
    public static double KiS = 0.000;
    public static double KdS = 0.0001;
    public static double KgS = .02;
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
        MotorController shoulderController = new MotorController();
        shoulderController.setConsistences(-shoulder.getCurrentPosition(), KpS, KiS, KdS, KgS);

        shoulderController.setMove(-shoulder.getCurrentPosition(), -shoulder.getCurrentPosition(), Acceleration, MaxVelocity);

        while (opModeIsActive()) {
            Map<String, Double> move = shoulderController.getMoveData();
            double shoulderTargetPos = shoulderController.calcCurrentTargetPos(move);
            shoulderController.setConsistences(-shoulder.getCurrentPosition(), KpS, KiS, KdS, KgS);

            telemetry.addData("P1", move.get("P1"));
            telemetry.addData("P2", move.get("P2"));
            telemetry.addData("P3", move.get("P3"));
            telemetry.addData("length", move.get("length"));
            telemetry.addData("moveEndTime", move.get("moveEndTime"));
            telemetry.addData("moveStartTime", move.get("moveStartTime"));
            telemetry.addData("startPos", move.get("startPos"));
            telemetry.addData("accel", move.get("accel"));
            telemetry.addData("peakVel", move.get("peakVel"));
            telemetry.addData("targetPos", shoulderTargetPos);
            telemetry.addData("currentPos", -shoulder.getCurrentPosition());

            double pow = shoulderController.calcPower(-shoulder.getCurrentPosition(), shoulderTargetPos);
            telemetry.addData("pow", pow);
            shoulder.setPower(pow);

            if (gamepad1.y) {
                shoulderController.setMove(-shoulder.getCurrentPosition(), 3000, Acceleration, MaxVelocity);
            } else if (gamepad1.a) {
                shoulderController.setMove(-shoulder.getCurrentPosition(), 1000, Acceleration, MaxVelocity);
            }
            telemetry.update();
        }
    }
}
