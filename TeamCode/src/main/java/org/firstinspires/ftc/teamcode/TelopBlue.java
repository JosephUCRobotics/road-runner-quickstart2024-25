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

public class TelopBlue extends LinearOpMode {
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, -Math.PI*.75));
        double rightFrontStart = drive.rightFront.getCurrentPosition();
        double leftFrontStart = drive.leftFront.getCurrentPosition();
        double rightBackStart = drive.rightBack.getCurrentPosition();
        double leftBackStart = drive.leftBack.getCurrentPosition();

        double[] robotXYA = {0, 0, 0};
        double targetAngle = drive.pose.heading.toDouble();

        ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
        Arm arm = new Arm(hardwareMap);
        DcMotor claw = hardwareMap.get(DcMotor.class, "claw");
        DcMotor hanger = hardwareMap.get(DcMotor.class, "hanger");
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanger.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo hook = hardwareMap.get(Servo.class, "hook");

        boolean lastGamepadX = false;
        boolean lastGamepadY = false;
        boolean lastGamepadA = false;
        boolean lastGamepadB = false;
        boolean lastRightBumper = false;
        //boolean lastDpadDown = false;
        boolean lastLeftBumper = false;
        //boolean lastDpadRight = false;

        double forwordDrivePow = 0;

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                if(!lastRightBumper){
                    moveArmToDropPos = !moveArmToDropPos;
                    moveArmToPickupPos = false;
                    moveArmToHookPoss = false;
                    moveArmToReadyPos = false;
                    reversIntake = false;
                }
                lastRightBumper = true;
            } else {
                lastRightBumper = false;
            }

//            if(gamepad1.dpad_down){
//                if(!lastDpadDown){
//                    moveArmToPickupPos = !moveArmToPickupPos;
//                    moveArmToDropPos = false;
//                    moveArmToHookPoss = false;
//                    moveArmToReadyPos = false;
//                    reversIntake = false;
//                }
//                lastDpadDown = true;
//            } else {
//                lastDpadDown = false;
//            }

            if(gamepad1.left_bumper){
                if(!lastLeftBumper){
                    moveArmToHookPoss = !moveArmToHookPoss;
                    moveArmToDropPos = false;
                    moveArmToPickupPos = false;
                    moveArmToReadyPos = false;
                    reversIntake = false;
                }
                lastLeftBumper = true;
            } else {
                lastLeftBumper = false;
            }

//            if(gamepad1.dpad_right){
//                if(!lastDpadRight){
//                    moveArmToReadyPos = !moveArmToReadyPos;
//                    moveArmToDropPos = false;
//                    moveArmToPickupPos = false;
//                    moveArmToHookPoss = false;
//                    reversIntake = false;
//                }
//                lastDpadRight = true;
//            } else {
//                lastDpadRight = false;
//            }

            if(gamepad1.a){
                if(!lastGamepadA){
                    targetColorOrYellowBlock = !targetColorOrYellowBlock;
                    targetColorBlock = false;
                    reversIntake = false;
                    
                    moveArmToPickupPos = !moveArmToPickupPos;
                    moveArmToDropPos = false;
                    moveArmToHookPoss = false;
                    moveArmToReadyPos = false;
                }
                lastGamepadA = true;
            } else {
                lastGamepadA = false;
            }

            if(gamepad1.b){
                if(!lastGamepadB){
                    targetColorBlock = !targetColorBlock;
                    reversIntake = false;
                    targetColorOrYellowBlock = false;

                    moveArmToPickupPos = !moveArmToPickupPos;
                    moveArmToDropPos = false;
                    moveArmToHookPoss = false;
                    moveArmToReadyPos = false;
                }
                lastGamepadB = true;
            } else {
                lastGamepadB = false;
            }

//            if(gamepad1.x){
//                if(!lastGamepadX){
//                    hookDown = !hookDown;
//                }
//                lastGamepadX = true;
//            } else {
//                lastGamepadX = false;
//            }
            
            if(gamepad1.x){
                if(!lastGamepadX){
                    moveArmToReadyPos = !moveArmToReadyPos;
                    moveArmToDropPos = false;
                    moveArmToPickupPos = false;
                    moveArmToHookPoss = false;
                }
                lastGamepadX = true;
            } else {
                lastGamepadX = false;
            }

            if(gamepad1.y){
                if(!lastGamepadY){
                    reversIntake = !reversIntake;
                    targetColorBlock = false;
                    targetColorOrYellowBlock = false;
                    dropPos = Math.abs(drive.pose.position.x) + Math.abs(drive.pose.position.y);
                }
                lastGamepadY = true;
            } else {
                lastGamepadY = false;
            }

            if (gamepad1.start){
                headingReset = drive.pose.heading.toDouble();
                targetAngle = drive.pose.heading.toDouble()-headingReset;
            }


            if (gamepad1.dpad_up){
                hanger.setPower(1);
            } else if (gamepad1.dpad_down) {
                hanger.setPower(-1);
            } else {
                hanger.setPower(0);
            }
            if (targetColorOrYellowBlock){
                claw.setPower(calcClawPow(color.red(), color.blue(), color.green(), 5));
            } else if (targetColorBlock) {
                claw.setPower(calcClawPow(color.red(), color.blue(), color.green(), 2));
            } else if (reversIntake) {
                hookDown = false;
                claw.setPower(-1);
                if (Math.abs(dropPos - (Math.abs(drive.pose.position.x) + Math.abs(drive.pose.position.y))) > 5){
                    moveArmToPickupPos = false;
                    moveArmToHookPoss = false;
                    moveArmToDropPos = false;
                    moveArmToReadyPos = true;
                }
            } else {
                claw.setPower(0);
            }

            if (moveArmToHookPoss){
                arm.moveArmToPoss(4127);
            } else if (moveArmToPickupPos) {
                arm.moveArmToPoss(12195);
            } else if (moveArmToReadyPos) {
                arm.moveArmToPoss(11609);
            } else if (moveArmToDropPos) {
                arm.moveArmToPoss(4525);
            } else {
                arm.moveArmManually(gamepad1.right_stick_y);
            }

            if (hookDown){
                hook.setPosition(.1);
            } else {
                hook.setPosition(0);
            }

            if(gamepad1.left_stick_button){
                forwordDrivePow = .2;
                if (arm.arm.getCurrentPosition()>2500){
                    forwordDrivePow *= -1;
                }
            } else {
                forwordDrivePow = 0;
            }

            robotXYA = calculateRobotXYA(-gamepad1.left_stick_y, -gamepad1.left_stick_x, (gamepad1.left_trigger - gamepad1.right_trigger), .7);
            if (robotXYA[2] != 0){
                targetAngle = drive.pose.heading.toDouble()-headingReset;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotXYA[0]+forwordDrivePow, robotXYA[1]-forwordDrivePow), robotXYA[2]));
            } else {
                if(Math.abs(targetAngle - (drive.pose.heading.toDouble()-headingReset)) > Math.PI/2){
                    if(targetAngle > 0){
                        targetAngle -= 2*Math.PI;
                    } else {
                        targetAngle += 2*Math.PI;
                    }
                }
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotXYA[0]+forwordDrivePow, robotXYA[1]-forwordDrivePow), (targetAngle - (drive.pose.heading.toDouble()-headingReset))*.3));
            }

            telemetry.addData("hangerPos", hanger.getCurrentPosition());
            telemetry.update();


            drive.updatePoseEstimate();

//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    public double[] calculateRobotXYA(double x, double y, double angle, double speed) {
        double currentAngle = drive.pose.heading.toDouble()-headingReset;
        // calculate exponential power
//        telemetry.addData("stick x", x);
//        telemetry.addData("stick y", y);
//        telemetry.addData("stick a", angle);

//        double a = .2;
//        x = a * (Math.pow((1+(a-.1))/a,Math.abs(x)))-(a-.1);
//        y = a * (Math.pow((1+(a-.1))/a,Math.abs(y)))-(a-.1);
//        angle = a * (Math.pow((1+(a-.1))/a,Math.abs(angle)))-(a-.1);

//        if (gamepad1.left_stick_x < 0){
//            stickXOut *= -1;
//        } else if (gamepad1.left_stick_x == 0) {
//            stickXOut = 0;
//        }

        // Driver orient the controls
        double rx = x*Math.cos(currentAngle) + y*Math.sin(currentAngle);
        double ry = y*Math.cos(currentAngle) - x*Math.sin(currentAngle);

        // Display wheel wanted robot x, y and angle speed
//        telemetry.addData("rx", rx);
//        telemetry.addData("ry", ry);
//        telemetry.addData("angle", Math.toDegrees(currentAngle));
//        telemetry.update();

        angle*=.8;
        rx*=speed;
        ry*=speed;
        return new double[]{rx, ry, angle};
    }
    public double calcClawPow(int red, int blue, int green, int targetColor/*1 red, 2 blue, 3 yellow, 4 red/yellow, 5 blue/yellow, 0 on*/){
//        telemetry.addData("red", red);
//        telemetry.addData("blue", blue);
//        telemetry.addData("green", green);
        int blockColor = 0;
        hookDown = false;
        if (green > 800 && red > 800){
            //yellow
            blockColor = 3;
        } else if (red > blue && red > 200) {
            //red
            blockColor = 1;
        } else if (blue > red && blue > 100) {
            //blue
            blockColor = 2;
        }

        double motorPow = 1;
        if (targetColor == blockColor){
            motorPow = 0;
            hookDown = true;
            targetColorBlock = false;
            targetColorOrYellowBlock = false;
            reversIntake = false;

            moveArmToPickupPos = false;
            moveArmToHookPoss = false;
            moveArmToDropPos = false;
            moveArmToReadyPos = true;
        }
        switch (targetColor){
            case 4:
                if (blockColor == 1 || blockColor == 3){
                    motorPow = 0;
                    hookDown = true;
                    targetColorBlock = false;
                    targetColorOrYellowBlock = false;
                    reversIntake = false;

                    moveArmToPickupPos = false;
                    moveArmToHookPoss = false;
                    moveArmToDropPos = false;
                    moveArmToReadyPos = true;
                }
                break;
            case 5:
                if (blockColor == 2 || blockColor == 3){
                    motorPow = 0;
                    hookDown = true;
                    targetColorBlock = false;
                    targetColorOrYellowBlock = false;
                    reversIntake = false;

                    moveArmToPickupPos = false;
                    moveArmToHookPoss = false;
                    moveArmToDropPos = false;
                    moveArmToReadyPos = true;
                }
                break;
        }

        return motorPow;
    }
    public class Arm {
        private DcMotorEx arm;
        public Arm(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            arm.setPower(pow);
            telemetry.addData("arm pos", arm.getCurrentPosition());
        }
    }
}
