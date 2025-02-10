package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

//import org.apache.commons.lang3.time.StopWatch;

@TeleOp(name = "TelopDualShoulderMotors", group = "A")
@Config
public class TelopDualShoulderMotors extends LinearOpMode {
    MecanumDrive drive;
    boolean moveArmToDropPos = false;
    boolean moveArmToPickupPos = false;
    boolean moveArmToReadyPos = false;
    boolean moveArmToHookPoss = false;
    double dropPos = 0;
    double headingReset = 0;
    int closeClaw = 0;
    boolean targetHangeClaw = false;

    boolean moveArmWithPath = false;
    boolean pathMoveSet = false;


    public static double elbowGravityCorection = .1;
    public static double shoulderGravityCorection = .1;
    public static double KpS = 0.002;
    public static double KiS = 0.000;
    public static double KdS = 0.0001;
    public static double GravityK = .02;
    public static double GravityK2 = 0.15;
    public static double maxPowUpS = 1;
    public static double maxPowDownS = 1;
    public static double KpE = .003;
    public static double KiE = .0000;
    public static double KdE = 0.0001;
    public static double maxPowUpE = .7;
    public static double maxPowDownE = .001;
    boolean lastWasJoystick = false;
    boolean clawTargetUp = true;
    boolean clawSpinLeft = false;
    boolean clawSpinRight = false;
    boolean shoulderTargetPosSet = false;
    boolean elbowTargetPosSet = false;
    boolean hangTimerSet = false;
    ElapsedTime hangTimer = new ElapsedTime();
    ElapsedTime moveToNextTime = new ElapsedTime();
    boolean moveToNextTimeUsed = true;
    boolean hangTimer2Set = false;
    ElapsedTime hangTimer2 = new ElapsedTime();
    int hangSegment = 0;
    ElapsedTime shoulderTimer = new ElapsedTime();
    PIDController shoulderController;
    private DcMotorEx shoulder;
    private DcMotorEx elbow;
    boolean posControl = false;
    boolean manualControl = false;

    public static double shoulderVelocetyCorectionK = 0;
    public static double shoulderFeedForwardDownCorrection = .75;
    public static double shoulderDownSpeed = .8;
    public static double KpE2 = 1;
    public static double shoulderFeedForwardOffsetK = .2;
    public static double shoulderFeadForwordK = 2;
    public static double shoulderCosFeadForwordK = 3;
    public static double KvE = 5;
    public static double KgS = .2;
    public static double elbowWeight = .05;
    public static double KgE = .0025;
    boolean hangManualy = false;
    boolean shoulderAtTarget = false;
    boolean elbowAtTarget = false;

    boolean clawDown = false;
    double manualClawPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, -Math.PI*.75));
        //shoulderController = new PIDController(1,0, 0);
        double rightFrontStart = drive.rightFront.getCurrentPosition();
        double leftFrontStart = drive.leftFront.getCurrentPosition();
        double rightBackStart = drive.rightBack.getCurrentPosition();
        double leftBackStart = drive.leftBack.getCurrentPosition();

        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        double[] robotXYA = {0, 0, 0};
        double targetAngle = drive.pose.heading.toDouble();

        Arm arm = new Arm(hardwareMap);

        boolean lastGamepadX = false;
        boolean lastGamepadY = false;
        boolean lastGamepadA = false;
        boolean lastGamepad2S = false;
        boolean lastGamepadB = false;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;
        boolean lastGamepad2A = false;
        boolean cutDrivePow = false;
        boolean lastG2Back = false;

        Map<String, Double> moveS = new HashMap<>();;
        Map<String, Double> moveE = new HashMap<>();;


        waitForStart();

        while (opModeIsActive()) {

            if(gamepad2.x){
                if(!lastRightBumper){
                    if (clawSpinLeft || clawSpinRight){
                        clawSpinLeft = false;
                        clawSpinRight = false;
                    } else {
                        clawSpinLeft = true;
                    }
                }
                lastRightBumper = true;
            } else {
                lastRightBumper = false;
            }
            if(gamepad2.back){
                if(!lastG2Back){
                    manualControl = !manualControl;
                }
                lastG2Back = true;
            } else {
                lastG2Back = false;
            }

            if(gamepad2.b){
                if(!lastLeftBumper){
                    if (clawSpinLeft || clawSpinRight){
                        clawSpinLeft = false;
                        clawSpinRight = false;
                    } else {
                        clawSpinRight = true;
                    }
                }
                lastLeftBumper = true;
            } else {
                lastLeftBumper = false;
            }

            if(gamepad1.back){
                if(!lastGamepadA){
                    cutDrivePow = !cutDrivePow;
                }
                lastGamepadA = true;
            } else {
                lastGamepadA = false;
            }
            if(gamepad2.start){
                if(!lastGamepad2S){
                    elbowAtTarget = false;
                    shoulderAtTarget = false;
                    hangSegment ++;
                }
                lastGamepad2S = true;
            } else {
                lastGamepad2S = false;
            }

            if(gamepad2.right_bumper || gamepad1.a){
                clawDown = false;
                if(!lastGamepad2A){
                    if(closeClaw != 0) {
                        closeClaw = 0;
                    } else {
                        closeClaw = 2;
                    }
                }
                lastGamepad2A = true;
            } else {
                lastGamepad2A = false;
            }



            if(gamepad2.left_bumper){
                clawDown = false;
                if(!lastGamepadB){
                    clawTargetUp = !clawTargetUp;
                    targetHangeClaw = false;
                }
                lastGamepadB = true;
            } else {
                lastGamepadB = false;
            }
            
            if(gamepad1.x){
                clawDown = true;
                if(!lastGamepadX){
                    if(closeClaw != 0) {
                        closeClaw = 0;
                    } else {
                        closeClaw = 1;
                    }
                }
                lastGamepadX = true;
            } else {
                lastGamepadX = false;
            }

//            if(gamepad1.y){
//                if(!lastGamepadY){
//                    dropPos = Math.abs(drive.pose.position.x) + Math.abs(drive.pose.position.y);
//                }
//                lastGamepadY = true;
//            } else {
//                lastGamepadY = false;
//            }


            if(targetHangeClaw){
                arm.targetHangPos();
                closeClaw = 2;
            } else {
                arm.levelClaw(clawTargetUp, manualClawPos, clawDown);
            }
            manualClawPos = gamepad2.left_trigger;
            arm.closeClaw(closeClaw);
            arm.spinClaw();

            if (gamepad1.start){
                headingReset = drive.pose.heading.toDouble()+Math.PI * .75;
                targetAngle = drive.pose.heading.toDouble()-headingReset;
            }

            if (gamepad2.a) {
                if (!pathMoveSet){
                    moveS = arm.setMove(-shoulder.getCurrentPosition(), 1455, 1500, 3000);
                    moveE = arm.setMove(-elbow.getCurrentPosition(), -5695, 1500, 3000);
                    pathMoveSet = true;
                    moveArmWithPath = true;
                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
                clawDown = false;

//            } else if (moveArmToPickupPos) {
//                arm.moveArmToPoss(12195);
//            } else if (moveArmToReadyPos) {
//                arm.moveArmToPoss(11609);
//            } else if (moveArmToDropPos) {
//                arm.moveArmToPoss(4525);
            } else if (gamepad2.right_trigger > .4) {
                if (!pathMoveSet){
                    moveS = arm.setMove(-shoulder.getCurrentPosition(), 3700, 1500, 3000);  // 3600
                    moveE = arm.setMove(-elbow.getCurrentPosition(), -4150, 1500, 3000);    //3800
                    pathMoveSet = true;
                    moveArmWithPath = true;
                }
                clawDown = false;
                if (Math.abs(3700 - (-shoulder.getCurrentPosition())) < 150 && Math.abs(-4150 - (-elbow.getCurrentPosition())) < 150){//ToDo this will audo drop
                    closeClaw = 2;
                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;

                manualClawPos = .4;
            } else if (gamepad2.y) {
                if (!pathMoveSet){
                    moveS = arm.setMove(-shoulder.getCurrentPosition(), 2700, 1500, 3000);
                    moveE = arm.setMove(-elbow.getCurrentPosition(), -5430, 1500, 3000);
                    pathMoveSet = true;
                    moveArmWithPath = true;
                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
                clawDown = false;
            } else if (gamepad1.y) {
                if (!pathMoveSet){
                    moveS = arm.setMove(-shoulder.getCurrentPosition(), 1550, 1500, 3000);
                    moveE = arm.setMove(-elbow.getCurrentPosition(), -4540, 1500, 3000);
                    pathMoveSet = true;
                    moveArmWithPath = true;
                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
                closeClaw = 1;
            } else if (gamepad2.dpad_up) {
                cutDrivePow = false;
                arm.shoulderTarget(2500);
                arm.elbowTarget(-4650, 1);

                elbowAtTarget = false;
                shoulderAtTarget = false;

                targetHangeClaw = true;

                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                hangTimerSet = false;
                hangTimer2Set = false;
                hangSegment = 0;
                hangManualy = false;
                clawDown = false;
            } else if (gamepad2.dpad_left) {
                cutDrivePow = true;
                hangTimer2Set = false;
                if (!hangTimerSet){
                    hangTimer.reset();
                }
                hangTimerSet = true;
                if (shoulderAtTarget && elbowAtTarget && moveToNextTimeUsed && hangSegment != 4) {
                    moveToNextTimeUsed = false;
                    moveToNextTime.reset();
                }
                if (moveToNextTime.seconds() > .3 && !moveToNextTimeUsed){
                    moveToNextTimeUsed = true;
                    elbowAtTarget = false;
                    shoulderAtTarget = false;
                    hangSegment ++;
                }

                if (hangSegment == 0){
                    arm.shoulderTargetForHanging(20);
                    arm.elbowTargetNoG(-4650, 1);
                } else if (hangSegment == 1) {
                    //cutDrivePow = true;
                    arm.shoulderTargetNoG(325);
                    arm.elbowTargetNoG(-200, 1);
                } else if (hangSegment == 2) {
                    arm.shoulderTargetNoG(1300);
                    arm.elbowTargetNoG(-200, 1);
                } else if (hangSegment == 3) {
                    arm.shoulderTargetNoG(1350);
                    arm.elbowTargetNoG(-3000, 1);
                } else if (hangSegment == 4) {
                    arm.shoulderTargetNoG(2415);//2375
                    arm.elbowTargetNoG(-5060, 1);  //5080
                } else if (hangSegment == 5) {
                    arm.shoulderTargetNoG(2360);
                    arm.elbowTargetNoG(-5500, 1);
                } else if (hangSegment == 6) {
                    arm.shoulderTargetForHanging(2850); //noG //2350
                    arm.elbowTargetNoG(-6400, 1);
                } else if (hangSegment == 7) {
                    arm.shoulderTargetForHanging(1500);
                    arm.elbowTargetNoG(-6400, 1);
                } else if (hangSegment == 8) {
                    arm.shoulderTargetForHanging(440);
                    arm.elbowTargetNoG(-5200, 1);
                }else if (hangSegment == 9) {
                    arm.shoulderTargetNoG(370);
                    arm.elbowTargetForHanging(-5700, 1);
                }
//                if (hangTimer.seconds() < 3){
//                    arm.shoulderTargetForHanging(50);
//                    arm.elbowTargetNoG(-4650, 1);
//                } else if (hangTimer.seconds() < 6.5) {
//                    //cutDrivePow = true;
//                    arm.shoulderTargetNoG(325);
//                    arm.elbowTargetNoG(-200, 1);
//                } else if (hangTimer.seconds() < 7.7) {
//                    arm.shoulderTargetNoG(1300);
//                    arm.elbowTargetNoG(-200, 1);
//                } else if (hangTimer.seconds() < 11) {
//                    arm.shoulderTargetNoG(1900);//2150
//                    arm.elbowTargetNoG(-4950, 1);
//                } else if (hangTimer.seconds() < 13) {
//                    arm.shoulderTargetNoG(2350);
//                    arm.elbowTargetNoG(-6450, 1);
//                } else if (hangTimer.seconds() < 14.3) {
//                    arm.shoulderTargetForHanging(1500);
//                    arm.elbowTargetNoG(-6450, 1);
//                } else if (hangTimer.seconds() < 16) {
//                    arm.shoulderTargetForHanging(400);
//                    arm.elbowTargetNoG(-5230, 1);
//                }else if (hangTimer.seconds() < 19) {
//                    arm.shoulderTargetNoG(150);
//                    arm.elbowTargetForHanging(-5700, 1);
//                }

                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                targetHangeClaw = true;
                hangManualy = false;
                clawDown = false;
            } else if (gamepad2.dpad_down) {
//                arm.shoulderTarget(325);
//                arm.elbowTarget(-200, 1);
//                elbowTargetPosSet = false;
//                shoulderTargetPosSet = false;
//                targetHangeClaw = true;
                hangManualy = true;

            } else if (gamepad2.dpad_right) {
                if (!hangTimer2Set){
                    hangTimer.reset();
                }
                hangTimer2Set = true;
                if (hangTimer2.seconds() < 3) {
                    arm.shoulderTarget(1650);
                    arm.elbowTarget(-5200, .3);
                } else if (hangTimer2.seconds() < 3) {
                    arm.shoulderTarget(2000);
                    arm.elbowTarget(-5200, .3);
                } else if (hangTimer2.seconds() < 5) {
                    arm.shoulderTarget(2085);
                    arm.elbowTarget(-5010, .3);
                } else if (hangTimer2.seconds() < 8) {
                    arm.shoulderTarget(2085);
                    arm.elbowTarget(-6627, 1);
                } else if (hangTimer2.seconds() < 123) {
                    arm.shoulderTarget(687);
                    arm.elbowTarget(-5804, 1);
                }
                hangManualy = false;

//                arm.shoulderTarget(1300);
//                arm.elbowTarget(-200, 1);
//                elbowTargetPosSet = false;
//                shoulderTargetPosSet = false;
//                targetHangeClaw = true;
            } else {
//                moveArmWithPath = false;
//                pathMoveSet = false;
//                lastWasJoystick = true;
//                double shoulderPos = -shoulder.getCurrentPosition();
//
//                if (-gamepad2.right_stick_y >= 0 && shoulderPos > 3700){
//                    arm.shoulderTarget(3700);
//                } else if (!hangManualy){
//                    arm.moveShoulderManually(-gamepad2.right_stick_y);
////                    if (manualControl){
////                        arm.moveShoulderManually(-gamepad2.right_stick_y);
////                        arm.moveElbowManually(-gamepad2.left_stick_y);
////                    } else {
//////                       arm.calcArmPowersWithStick();
////                        arm.moveShoulder(-gamepad2.right_stick_y);
////                        arm.moveElbow(-gamepad2.left_stick_y);
////                    }
//                }
//                if (-gamepad2.left_stick_y >= 0 && shoulderPos > 2300 && -elbow.getCurrentPosition() > -3800) {
//                    arm.elbowTarget( -3800, 1);
//                } else if (!hangManualy) {
//                    arm.moveElbowManually(-gamepad2.left_stick_y + (-gamepad1.right_stick_y));
//                }
                arm.calcArmPowersWithStick();
            }



            if (moveArmWithPath) {
                double shoulderTarget = arm.calcCurrentTargetPos(moveS);
                double elbowTarget = arm.calcCurrentTargetPos(moveE);

                arm.shoulderTargetForPath(shoulderTarget);
                arm.elbowTargetForPath(elbowTarget);
            }

            if (!cutDrivePow && !hangManualy){
                robotXYA = calculateRobotXYA(-gamepad1.left_stick_y, -gamepad1.left_stick_x, (gamepad1.left_trigger - gamepad1.right_trigger), .7);
                if (robotXYA[2] != 0){
                    targetAngle = drive.pose.heading.toDouble()-headingReset;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotXYA[0], robotXYA[1]), robotXYA[2]));
                } else {
                    if(Math.abs(targetAngle - (drive.pose.heading.toDouble()-headingReset)) > Math.PI/2){
                        if(targetAngle > 0){
                            targetAngle -= 2*Math.PI;
                        } else {
                            targetAngle += 2*Math.PI;
                        }
                    }
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotXYA[0], robotXYA[1]), (targetAngle - (drive.pose.heading.toDouble()-headingReset))*.3));
                }
            }
            if (hangManualy){
                shoulder.setPower(gamepad1.left_stick_y);
                elbow.setPower(gamepad1.right_stick_y);
            }

            // arm.checkArmsPos();

            telemetry.addData("","");

            telemetry.addData("hangSegment" ,hangSegment);

            telemetry.addData("hang manualy",hangManualy);
            telemetry.addData(" cutDrivePow",cutDrivePow);

            telemetry.update();

            drive.updatePoseEstimate();
        }
    }
    public double[] calculateRobotXYA(double x, double y, double angle, double speed) {
        double currentAngle = drive.pose.heading.toDouble()-headingReset;

        // Driver orient the controls
        double rx = x*Math.cos(currentAngle) + y*Math.sin(currentAngle);
        double ry = y*Math.cos(currentAngle) - x*Math.sin(currentAngle);

        angle*=.8;
        rx*=speed;
        ry*=speed;
        return new double[]{rx, ry, angle};
    }

    public class Arm {
        private DcMotorEx shoulder;
        private DcMotorEx elbow;
        Servo clawClose;
        Servo clawSpin;
        Servo clawUp;
        public Arm(HardwareMap hardwareMap){
            shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
            //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

            elbow = hardwareMap.get(DcMotorEx.class, "elbow");
            //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);

            clawClose = hardwareMap.get(Servo.class, "clawClose");
            clawSpin = hardwareMap.get(Servo.class, "clawSpin");
            clawUp = hardwareMap.get(Servo.class, "clawUp");
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
            telemetry.addData("P1", P1);
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

        public void closeClaw(int close){
            if (close == 0){
                clawClose.setPosition(.45);
            } else if (close == 1){
                    clawClose.setPosition(.34);
            } else {
                clawClose.setPosition(.2);
            }
        }
        public void targetHangPos(){
            clawUp.setPosition(.5);
        }
        public void spinClaw(){
            if (clawSpinLeft) {
                clawSpin.setPosition(0);
            } else if (clawSpinRight) {
                clawSpin.setPosition(1);
            } else {
                clawSpin.setPosition(.42);
            }
        }
        double endArmAngle;
        double servoTarget;
        public void levelClaw(boolean flat, double pos, boolean down){
            if (pos ==0 || -elbow.getCurrentPosition() > -4144 && -elbow.getCurrentPosition() < -2619 || -shoulder.getCurrentPosition() < 3000){
                endArmAngle = ((-shoulder.getCurrentPosition() - 1457)/8192.0 * 2*Math.PI)+ ((-elbow.getCurrentPosition() + 3344)/8192.0 * 2*Math.PI);
                servoTarget = -endArmAngle/Math.PI*.75+.04;  // +.11
                if (-elbow.getCurrentPosition() > -4144 && -elbow.getCurrentPosition() < -2619) {
                    flat = false;
                }
                if (!flat){
                    servoTarget+=.4;
                }
                if (servoTarget < .11){
                    servoTarget = .11;
                }
                if (servoTarget > .69){
                    servoTarget = .69;
                }
                if (down){
                    servoTarget = .15;
                }
            } else {
                servoTarget = pos * .69; //69
            }
//            telemetry.addData("servo target: ", servoTarget);
//            telemetry.addData("servo current: ", clawUp.getPosition());
            clawUp.setPosition(servoTarget);

        }


        double integralSumS = 0;
        double integralSumE = 0;
        double lastErrorS = 0;
        double lastErrorE = 0;
        ElapsedTime timerE = new ElapsedTime();
        ElapsedTime timerS = new ElapsedTime();
        double encoderPosition;
        double error;
        double derivative;
        double out;
        double oldShoulderTarget = 0;
        public void setIntegralSumSTo0() {
            integralSumS = 0;
        }
        public void shoulderTarget(double reference) {
            double maxPowDownS = 1;
            // obtain the encoder position
            encoderPosition = -shoulder.getCurrentPosition();
            //telemetry.addData("shoulder pos: ", encoderPosition);

            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * GravityK + elbowGravity;

            out = (KpS * error) + (KiS * integralSumS) + (KdS * derivative);

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



//            if (Math.abs(error) < 20) {
//                moveArmToDropPos = false;
//                moveArmToHookPoss = false;
//                moveArmToPickupPos = false;
//                moveArmToReadyPos = false;
//            }
        }

        public void shoulderTargetNoG(double reference) {
            double maxPowDownS = 1;
            // obtain the encoder position
            encoderPosition = -shoulder.getCurrentPosition();
            //telemetry.addData("shoulder pos: ", encoderPosition);

            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * GravityK + elbowGravity;

            out = (KpS * error) + (KiS * integralSumS) + (KdS * derivative);

            out = Math.max(Math.min(out, 1), -1);

            //out += gravity;

//            if (out > maxPowUpS){
//                out = maxPowUpS;
//            } else if (out < - maxPowDownS){
//                out = -maxPowDownS;
//            }

            shoulder.setPower(out);

            if (Math.abs(error) < 200){
                shoulderAtTarget = true;
            } else {
                shoulderAtTarget = false;
            }

            lastErrorS = error;
            // reset the timer for next time
            timerS.reset();



//            if (Math.abs(error) < 20) {
//                moveArmToDropPos = false;
//                moveArmToHookPoss = false;
//                moveArmToPickupPos = false;
//                moveArmToReadyPos = false;
//            }
        }
        public void shoulderTargetForHanging(double reference) {
            double maxPowDownS = 1;
            // obtain the encoder position
            encoderPosition = -shoulder.getCurrentPosition();
            //telemetry.addData("shoulder pos: ", encoderPosition);

            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * GravityK + elbowGravity;

            out = (.1 * error);

            out = Math.max(Math.min(out, 1), -1);

//            if (out > maxPowUpS){
//                out = maxPowUpS;
//            } else if (out < - maxPowDownS){
//                out = -maxPowDownS;
//            }

            shoulder.setPower(out);

            if (Math.abs(error) < 200){
                shoulderAtTarget = true;
            }  else if (Math.abs(error) > 200){
                shoulderAtTarget = false;
            }

            lastErrorS = error;

            // reset the timer for next time
            timerS.reset();
        }
        double elbowGravity = 0;
        public void elbowTarget( double reference, double maxPowDownE) {
            // obtain the encoder position
            encoderPosition = -elbow.getCurrentPosition();
            //telemetry.addData("elbow pos: ", encoderPosition);
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            double elbowAngle = ((encoderPosition + 3344)/ticksPerRev)*(2*Math.PI) + shoulderAngle;

            elbowGravity = Math.cos(elbowAngle) * GravityK2;

            out = (KpE * error) + (KiE * integralSumE) + (KdE * derivative) + elbowGravity;

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
        public void elbowTargetNoG( double reference, double maxPowDownE) {
            // obtain the encoder position
            encoderPosition = -elbow.getCurrentPosition();
            //telemetry.addData("elbow pos: ", encoderPosition);
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            double elbowAngle = ((encoderPosition + 3344)/ticksPerRev)*(2*Math.PI) + shoulderAngle;

            elbowGravity = Math.cos(elbowAngle) * GravityK2;

            out = (KpE * error) + (KiE * integralSumE) + (KdE * derivative);

            out = Math.max(Math.min(out, 1), -1);
//            if (out > maxPowUpE){
//                out = maxPowUpE;
//            } else if (out < - maxPowDownE){
//                out = -maxPowDownE;
//            }

            elbow.setPower(out);

            if (Math.abs(error) < 200){
                elbowAtTarget = true;
            }  else if (Math.abs(error) > 200){
                shoulderAtTarget = false;
            }

            lastErrorE = error;

            // reset the timer for next time
            timerE.reset();

            telemetry.addData("elbowAngle", elbowAngle);

//            if (Math.abs(error) < 20) {
//                moveArmToDropPos = false;
//                moveArmToHookPoss = false;
//                moveArmToPickupPos = false;
//                moveArmToReadyPos = false;
//            }
        }
        public void elbowTargetForHanging( double reference, double maxPowDownE) {
            // obtain the encoder position
            encoderPosition = -elbow.getCurrentPosition();
            //telemetry.addData("elbow pos: ", encoderPosition);
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            double elbowAngle = ((encoderPosition + 3344)/ticksPerRev)*(2*Math.PI) + shoulderAngle;

            elbowGravity = Math.cos(elbowAngle) * GravityK2;

            out = (.01 * error);

            out = Math.max(Math.min(out, 1), -1);
//            if (out > maxPowUpE){
//                out = maxPowUpE;
//            } else if (out < - maxPowDownE){
//                out = -maxPowDownE;
//            }

            elbow.setPower(out);

            lastErrorE = error;
            if (Math.abs(error) < 200){
                elbowAtTarget = true;
            }  else if (Math.abs(error) > 200){
                shoulderAtTarget = false;
            }

            // reset the timer for next time
            timerE.reset();

        }
        int ticksPerRev = 8192;
        double shoulderAngle = 0;

        public void shoulderTargetForPath(double reference) {
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

            double gravity = Math.cos(shoulderAngle) * .02 + elbowGravity;

            double out = (0.002 * error) + (0 * integralSumS) + (0.0001 * derivative);

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

        double GravityKE = 0.15;
        public void elbowTargetForPath( double reference) {
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

            double out = (.003 * error) + (0 * integralSumE) + (0.0001 * derivative) + elbowGravity;

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

        double shoulderTargetPos = 0;

        public void moveShoulderManually(double pow){
            if (!shoulderTargetPosSet){
                shoulderTargetPos = -shoulder.getCurrentPosition();
                shoulderTargetPosSet = true;
            }

            if (pow!=0){
                shoulder.setPower(pow*.6);
                shoulderTargetPosSet = false;
            } else {
                shoulderTarget(shoulderTargetPos);
            }

//            telemetry.addData("shoulder pos", -shoulder.getCurrentPosition());
//            telemetry.addData("shoulder target pos", shoulderTargetPos);


        }
        double elbowTargetPos = 0;

        public void moveElbowManually(double pow){
            if (!elbowTargetPosSet){
                elbowTargetPos = -elbow.getCurrentPosition();
                elbowTargetPosSet = true;
            }

            if (pow!=0){
                elbow.setPower(pow*.6);
                elbowTargetPosSet = false;
            } else {
                elbowTarget(elbowTargetPos, .7);
            }

            //telemetry.addData("elbow pos", -elbow.getCurrentPosition());

        }

        public double[][] invert(double a[][])
        {
            int n = a.length;
            double x[][] = new double[n][n];
            double b[][] = new double[n][n];
            int index[] = new int[n];
            for (int i=0; i<n; ++i)
                b[i][i] = 1;

            // Transform the matrix into an upper triangle
            gaussian(a, index);

            // Update the matrix b[i][j] with the ratios stored
            for (int i=0; i<n-1; ++i)
                for (int j=i+1; j<n; ++j)
                    for (int k=0; k<n; ++k)
                        b[index[j]][k]
                                -= a[index[j]][i]*b[index[i]][k];

            // Perform backward substitutions
            for (int i=0; i<n; ++i)
            {
                x[n-1][i] = b[index[n-1]][i]/a[index[n-1]][n-1];
                for (int j=n-2; j>=0; --j)
                {
                    x[j][i] = b[index[j]][i];
                    for (int k=j+1; k<n; ++k)
                    {
                        x[j][i] -= a[index[j]][k]*x[k][i];
                    }
                    x[j][i] /= a[index[j]][j];
                }
            }
            return x;
        }

// Method to carry out the partial-pivoting Gaussian
// elimination.  Here index[] stores pivoting order.

        public void gaussian(double a[][], int index[])
        {
            int n = index.length;
            double c[] = new double[n];

            // Initialize the index
            for (int i=0; i<n; ++i)
                index[i] = i;

            // Find the rescaling factors, one from each row
            for (int i=0; i<n; ++i)
            {
                double c1 = 0;
                for (int j=0; j<n; ++j)
                {
                    double c0 = Math.abs(a[i][j]);
                    if (c0 > c1) c1 = c0;
                }
                c[i] = c1;
            }

            // Search the pivoting element from each column
            int k = 0;
            for (int j=0; j<n-1; ++j)
            {
                double pi1 = 0;
                for (int i=j; i<n; ++i)
                {
                    double pi0 = Math.abs(a[index[i]][j]);
                    pi0 /= c[index[i]];
                    if (pi0 > pi1)
                    {
                        pi1 = pi0;
                        k = i;
                    }
                }

                // Interchange rows according to the pivoting order
                int itmp = index[j];
                index[j] = index[k];
                index[k] = itmp;
                for (int i=j+1; i<n; ++i)
                {
                    double pj = a[index[i]][j]/a[index[j]][j];

                    // Record pivoting ratios below the diagonal
                    a[index[i]][j] = pj;

                    // Modify other elements accordingly
                    for (int l=j+1; l<n; ++l)
                        a[index[i]][l] -= pj*a[index[j]][l];
                }
            }
        }
        public  double[][] multiplyMatrices(double[][] matrix1, double[][] matrix2) {
            int rows1 = matrix1.length;
            int cols1 = matrix1[0].length;
            int cols2 = matrix2[0].length;

            if (cols1 != matrix2.length) {
                throw new IllegalArgumentException("Invalid matrix dimensions for multiplication.");
            }

            double[][] result = new double[rows1][cols2];

            for (int i = 0; i < rows1; i++) {
                for (int j = 0; j < cols2; j++) {
                    for (int k = 0; k < cols1; k++) {
                        result[i][j] += matrix1[i][k] * matrix2[k][j];
                    }
                }
            }

            return result;
        }


        int ticksPerMotorRev = 8192;
        double gearRatio = 1;
        //double ticksPerRev = ticksPerMotorRev * gearRatio;
        double maxVelocity = 0.125;
        //double shoulderAngle = 0;

        public void moveShoulder(double powIn){
            double direction = 1;
            if (powIn < 0){
                direction = -1;
            }
            telemetry.addData("direction", direction);
            double vel = -shoulder.getVelocity() / ticksPerRev;

            double pos = -shoulder.getCurrentPosition() ;

            shoulderAngle = ((pos - 1350)/ticksPerRev)*(2*Math.PI);

            double shoulderCosAngle = Math.cos(shoulderAngle);

            double desiredVelocity = powIn*maxVelocity;

            //double pow = cosAngle * (KvS*desiredVelocity + KgS) + Kv2S*desiredVelocity;
        /*if (pow == 0){
            pow = .00001;
        }*/
            double feedForeword = ((direction * shoulderCosAngle + shoulderFeedForwardOffsetK) * Math.abs(desiredVelocity)  * shoulderCosFeadForwordK) + desiredVelocity * shoulderFeadForwordK;
            if (vel < desiredVelocity * shoulderDownSpeed && direction < 0) {
                feedForeword *= shoulderFeedForwardDownCorrection;
            }
            double shoulderVelocityErrCorrection = shoulderVelocetyCorectionK * (desiredVelocity - vel);

            double elbowAngle = (-elbow.getCurrentPosition() + 3300)/8192.0 * 2*Math.PI;
            double elbowCosAngle = Math.cos(shoulderAngle + elbowAngle);
            double gravityCorrection = KgS * shoulderCosAngle + elbowCosAngle * elbowWeight;

            double pow = gravityCorrection + feedForeword + shoulderVelocityErrCorrection;

            //double pow = shoulderController.output(-gamepad1.left_stick_y*maxVelocity, vel,-gamepad1.left_stick_y*maxVelocity);


            //lastPos = pos;
            //lastTime = time;
            //lastPow = pow;



            //pow += angleMultiplyer;
            //pow += shoulderGravityCorection * angleMultiplyer;

            shoulder.setPower(pow);
            telemetry.addData("shoulder raw vel", -shoulder.getVelocity());
            telemetry.addData("shoulder rev/sec vel", vel);
            telemetry.addData("shoulder desiredVelocity", desiredVelocity);
            telemetry.addData("Shoulder target Pow", pow);
            telemetry.addData("shoulder pos", pos);
            telemetry.addData("shoulder angle(RAD)", shoulderAngle);
            telemetry.addData("elbow angle", elbowAngle);
            telemetry.addData("shoulder Cos(angle)", shoulderCosAngle);
            telemetry.addData("shoulder feedForeword", feedForeword);
            telemetry.addData("shoulder shoulderVelocityErrCorrection", shoulderVelocityErrCorrection);
            telemetry.addData("shoulder gravity correction", KgS * shoulderCosAngle);
            telemetry.addData("shoulderFeedForwardDownCorrection", shoulderFeedForwardDownCorrection);
            telemetry.addData("gravityCorrection", gravityCorrection);

            telemetry.addData("","");



        }

        public void moveElbow(double powIn){
            double vel = -elbow.getVelocity() / ticksPerRev;

            double pos = -elbow.getCurrentPosition() + ticksPerRev;

            double cosAngle = Math.cos((pos/ticksPerRev)*(2*Math.PI)+shoulderAngle);

            double desiredVelocity = powIn*maxVelocity;

            //double pow = cosAngle * (KvS*desiredVelocity + KgS) + Kv2S*desiredVelocity;
        /*if (pow == 0){
            pow = .00001;
        }*/
            double pow = KgE * cosAngle + KvE * desiredVelocity + KpE2 * (desiredVelocity - vel);

            //double pow = shoulderController.output(-gamepad1.left_stick_y*maxVelocity, vel,-gamepad1.left_stick_y*maxVelocity);


            //lastPos = pos;
            //lastTime = time;
            //lastPow = pow;



            //pow += angleMultiplyer;
            //pow += shoulderGravityCorection * angleMultiplyer;

            elbow.setPower(pow);
            telemetry.addData("elbow raw vel", -elbow.getVelocity());
            telemetry.addData("elbow rev/sec vel", vel);
            telemetry.addData("elbow desiredVelocity", desiredVelocity);
            telemetry.addData("elbow target pow", pow);
            telemetry.addData("elbow pos", pos);
            telemetry.addData("elbow angle(RAD)", (pos/ticksPerRev)*(2*Math.PI));
            telemetry.addData("elbow Cos(angle)", cosAngle);


            //telemetry.update();

        }

        double arm1Length= 15 + 3/16;
        double arm2Length = 13.75;
        double[] angle = new double[2];
        double[] startPos = {255, 255};
        double[] v = new double[2];
        boolean bendUp = false;
        double minBend = .3;
        double maxBend = 2.5;
        boolean armIsAtMin = false;
        double joint1MaxUp = 1.7;
        double joint1MaxDown = -1;
        double[][] inverseJ = new double[2][1];
        double[] changeInA = new double[2];
        boolean inUpperHalf;
        boolean aDown2 = false;

        void calcArmPowersWithStick(){
            angle[0] = (-shoulder.getCurrentPosition() - 1457)/8192.0 * 2*Math.PI;
            angle[1] = (-elbow.getCurrentPosition() + 3344)/8192.0 * 2*Math.PI;
            telemetry.addData("angle0", angle[0]);
            telemetry.addData("angle1", angle[1]);

            if (angle[1]-angle[0]>0){
                bendUp = false;
            } else {
                bendUp = true;
            }
            // Calculate matrix J
            double[][] j = new double[2][2];
            j[0][0] = -arm1Length*Math.sin(angle[0]) -arm2Length*Math.sin(angle[0] + angle[1]);
            j[0][1] = -arm2Length*Math.sin(angle[0] + angle[1]);
            j[1][0] = arm1Length*Math.cos(angle[0]) + arm2Length*Math.cos(angle[0] + angle[1]);
            j[1][1] = arm2Length*Math.cos(angle[0] + angle[1]);

            if (v[0]!=0 || v[1]!=0){
                inverseJ = invert(j);
                double[][] newV = new double[2][1];
                newV[0][0] = v[0];
                newV[1][0] = v[1];
                double[][] newChangeInA = multiplyMatrices(inverseJ, newV);
                changeInA[0] = newChangeInA[0][0];
                changeInA[1] = newChangeInA[1][0];
            } else {
                changeInA = new double[2];
            }

            changeInA[0] /= 2;
            changeInA[1] /= 2;

            angle[0] = angle[0] + changeInA[0];
            angle[1] = angle[1] + changeInA[1];

            double[] pos1 = new double[2];
            pos1[0] = Math.cos(angle[0])*arm1Length;
            pos1[1] = Math.sin(angle[0])*arm1Length;
            double[] pos2 = new double[2];
            pos2[0] = Math.cos(angle[0] + angle[1])*arm2Length;
            pos2[1] = Math.sin(angle[0] + angle[1])*arm2Length;

            v[0] = -gamepad2.left_stick_y;
            v[1] = -gamepad2.right_stick_y;

            // check to see if the arm is in the upper half or the lower half
            if (angle[0] >= 0) {
                inUpperHalf = true;
            } else {
                inUpperHalf = false;
            }

            telemetry.addData("BendUp ", bendUp);


            armIsAtMin = false;
            if (bendUp){
                if (angle[1] >= -minBend) {
                    if (angle[1] >= -minBend + .02) {
                        angle[1] -= .02;
                    } else {
                        angle[1] = -minBend;
                    }
                    armIsAtMin = true;
                } else if (angle[1] <= -maxBend) {
                    angle[1] = -maxBend;
                    if (v[0] < 0){
                        v[0] = 0;
                    }
                    if (v[1] > 0){
                        v[1] = 0;
                    }
                }
            } else {
                if (angle[1] <= minBend) {
                    if (angle[1] <= minBend - .02) {
                        angle[1] += .02;
                    } else {
                        angle[1] = minBend;
                    }
                    armIsAtMin = true;
                } else if (angle[1] >= maxBend) {
                    angle[1] = maxBend;
                    if (angle[0] >= Math.PI/2) {
                        if (v[0] > 0) {
                            v[0] = 0;
                        }
                        if (v[1] > 0){
                            v[1] = 0;
                        }
                    }else {
                        if (v[0]<0) {
                            v[0] = 0;
                        }
                        if (v[1]<0) {
                            v[1] = 0;
                        }
                    }
                }
            }
            if (armIsAtMin) {
                telemetry.addData("At Min", "True");
                if (v[0] > 0){
                    v[0] = 0;
                }
                if (inUpperHalf) {
                    if (v[1] > 0) {
                        v[1] = 0;
                    }
                } else {
                    if (v[1] < 0) {
                        v[1] = 0;
                    }
                }
            }
            if (angle[0] > joint1MaxUp) {
                angle[0] = joint1MaxUp;
            } else if (angle[0] < joint1MaxDown) {
                angle[0] = joint1MaxDown;
            }

            if (pos1[0] + pos2[0] > 27) {
                if (v[0] > 0) {
                    v[0] = 0;
                }
            }
            if (pos1[1] + pos2[1] < -13.75) {
                if (v[1] < 0) {
                    v[1] = 0;
                }
            }
            if (pos1[1] + pos2[1] < 0 && pos1[0] + pos2[0] < 6){
                if (v[0] < 1){
                    v[0] = 0;
                }
            }
            if (pos1[1] + pos2[1] > 0 && pos1[0] + pos2[0] < 0){
                if (v[0] < 0){
                    v[0] = 0;
                }
            }
            telemetry.addData("powerS", changeInA[0]*15);
            telemetry.addData("powerE", changeInA[1]*15);
            moveShoulder(changeInA[0]*15);
            moveElbow(changeInA[1]*15);

        }
        void checkArmsPos(){
            double shoulderPos = -shoulder.getCurrentPosition();
//            telemetry.addData("shoulder", shoulderPos);
//            telemetry.addData("elbow", -elbow.getCurrentPosition());
//            telemetry.addData("pow", shoulder.getPower());
            if (shoulderPos > 2300 && -elbow.getCurrentPosition() > -2550){
                if (shoulder.getPower() >= 0){
                    shoulder.setPower(-1);
                }
            }

            if (shoulderPos > 3300){
                if (shoulder.getPower() >= 0){
                    shoulder.setPower(-1);
                }
            }
        }
    }

}
