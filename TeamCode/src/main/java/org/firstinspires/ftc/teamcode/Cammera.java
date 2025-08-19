package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

// adb connect 192.168.43.1:5555
@Autonomous(name = "Camera", group = "A")
public final class Cammera extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        Load the OpenCV native library

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraVariables.hardwareMap = hardwareMap;
        MainVision vision = new MainVision();
        vision.setManualExposure(CameraVariables.exposure, CameraVariables.gain);
        telemetry.addData("file path: ", "go");
        telemetry.update();

//
//        while (opModeInInit()) {
//            vision.setManualExposure(CameraVariables.exposure, CameraVariables.gain);
//            double angle1 = -CameraVariables.distanceX1 / 14 + 90;
//            double angle2 = CameraVariables.distanceX2 / 14 + 90;
//            double c = 3;
//            double C = 180 - angle1 - angle2;
//            double a = Math.sin(Math.toRadians(angle1)) * (c/Math.sin(Math.toRadians(C)));
//
//            double z = (Math.sin(Math.toRadians(angle2))*a)*.8-17;
//
//            double angle = angle2-angle1;
//            telemetry.addData("X1: ", angle1);
//            telemetry.addData("X2: ", angle2);
//            telemetry.addData("Z: ", z);
//            telemetry.addData("angle: ", angle);
//            telemetry.addData("file path: ", vision.saveMatsAsJPG());
//            telemetry.update();
//        }

        waitForStart();
        telemetry.addData("file path: ", vision.saveMatsAsJPG("1"));
        telemetry.update();
        sleep(5000);
    }
}


