package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@TeleOp(name = "Move Arm Maunaly", group = "Manual Control")
@Config
public class moveArmManualy extends LinearOpMode {

    public static double Kp = .01;
    public static double Ki = .000001;
    public static double Kd = 0.0001;
    public static double maxPowUp = .7;
    public static double maxPowDown = .001;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                arm.targetPos(500);
            } else if (gamepad1.b) {
                arm.targetPos(0);
            } else {
                arm.stopArm();
            }
            arm.moveArmManually(-gamepad1.left_stick_y, gamepad1.right_stick_x);
//            telemetry.addData("Y Pow: ", gamepad1.left_stick_y);
//            telemetry.addData("X Pow: ", gamepad1.right_stick_x);
//            telemetry.update();
        }
    }

    public class Arm {
        private DcMotorEx shoulder;
        private DcMotorEx elbow;
        double theta1;
        double theta2;
        double R1 = 15 + 3/16;
        double R2 = 13.75;
        double tixPerRev = 8192;




        double integralSum = 0;
        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();
        double encoderPosition;
        double error;
        double derivative;
        double out;
        public Arm(HardwareMap hardwareMap){
            shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

            elbow = hardwareMap.get(DcMotorEx.class, "elbow");
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public void moveElbowToPoss(double targetPos){
            double currentPos = -elbow.getCurrentPosition();
            double stopErr = 50;
            if (currentPos < targetPos-stopErr) {
                elbow.setPower(.4);
            } else if (currentPos > targetPos+stopErr) {
                elbow.setPower(-.4);
            } else {
                elbow.setPower(0);
            }

//            if (targetPos == 4000 && arm.getCurrentPosition() > 3000){
//                targetColorOrYellowBlock = true;
//                targetColorBlock = false;
//                reversIntake = false;
//
//            }
        }

        double[][] matrixJ = new double[2][2];
        public void moveArmManually(double yPow, double xPow){
            theta1 = (-shoulder.getCurrentPosition())/tixPerRev * 2*Math.PI;
            theta2 = (-elbow.getCurrentPosition())/tixPerRev * 2*Math.PI;
            matrixJ[0][0] = -R1*Math.sin(theta1) - R2*Math.sin(theta1 + theta2);
            matrixJ[1][0] = R1*Math.cos(theta1)+R2*Math.cos(theta1 + theta2);
            matrixJ[0][1] = -R2*Math.sin(theta1 + theta2);
            matrixJ[1][1] = R2*Math.cos(theta1 + theta2);
            double c = 15;

            telemetry.addData("Current Theta 1: ", theta1);
            telemetry.addData("Current Theta 2: ", theta2);
            telemetry.addData("Theta 2 Encoder: ", -elbow.getCurrentPosition());
            telemetry.addData("Current Theta 2: ", -elbow.getCurrentPosition()-theta1);

//            if (go){
//                shoulder.setPower(yPow);
//                moveElbowToPoss(theta2/2*Math.PI);
//            }

        }
        public void  targetPos(double reference) {
            // obtain the encoder position
            encoderPosition = -shoulder.getCurrentPosition();
            telemetry.addData("pos: ", encoderPosition);
            telemetry.update();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            if (out > maxPowUp){
                out = maxPowUp;
            } else if (out < - maxPowDown){
                out = -maxPowDown;
            }

            shoulder.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();
        }
        public void stopArm(){
            shoulder.setPower(0);
            elbow.setPower(0);
        }
    }
}
