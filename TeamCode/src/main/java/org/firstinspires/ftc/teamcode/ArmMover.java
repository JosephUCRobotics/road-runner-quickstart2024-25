package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmMover extends LinearOpMode {
    MecanumDrive drive;
    boolean hookDown = false;
    boolean moveArmToDropPos = false;
    boolean moveArmToPickupPos = false;
    boolean moveArmToReadyPos = false;
    boolean moveArmToHookPoss = false;
    boolean reversIntake = false;
    boolean targetColorBlock = false;
    boolean targetColorOrYellowBlock = false;
    double dropPos = 0;
    double headingReset = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);


        double forwordDrivePow = 0;

        waitForStart();

        while (opModeIsActive()) {
            arm.moveArmManually(gamepad1.right_stick_y*1000);
        }}
    public class Arm {
        private DcMotorEx arm;
        public Arm(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void moveArmToPoss(double targetPos){
            double currentPos = arm.getCurrentPosition();
            double stopErr = 100;
            if (currentPos < targetPos-stopErr) {
                arm.setPower(1);
            } else if (currentPos > targetPos+stopErr) {
                arm.setPower(-1);
            } else {
                arm.setPower(0);
                moveArmToDropPos = false;
                moveArmToHookPoss = false;
                moveArmToPickupPos = false;
                moveArmToReadyPos = false;
            }

//            if (targetPos == 4000 && arm.getCurrentPosition() > 3000){
//                targetColorOrYellowBlock = true;
//                targetColorBlock = false;
//                reversIntake = false;
//
//            }
        }
        public void moveArmManually(double pow){
            arm.setVelocity(pow);
            telemetry.addData("arm pos", arm.getCurrentPosition());
        }
    }
}
