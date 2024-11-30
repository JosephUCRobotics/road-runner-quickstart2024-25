package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name = "Reset Encoders", group = "A")
public class resetEncoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
    }
}
