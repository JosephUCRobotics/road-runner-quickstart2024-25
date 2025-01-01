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

@TeleOp(name = "Telop", group = "A")
@Config
public class Telop extends LinearOpMode {
    MecanumDrive drive;
    boolean moveArmToDropPos = false;
    boolean moveArmToPickupPos = false;
    boolean moveArmToReadyPos = false;
    boolean moveArmToHookPoss = false;
    double dropPos = 0;
    double headingReset = 0;
    boolean closeClaw = false;
    boolean targetHangeClaw = false;


    public static double elbowGravityCorection = .1;
    public static double shoulderGravityCorection = .1;
    public static double KpS = 0.0005;
    public static double KiS = 0.000;
    public static double KdS = 0.000;
    public static double GravityK = .15;
    public static double maxPowUpS = 1;
    public static double maxPowDownS = 1;
    public static double KpE = .002;
    public static double KiE = .00002;
    public static double KdE = 0.0003;
    public static double maxPowUpE = .7;
    public static double maxPowDownE = .001;
    boolean lastWasJoystick = false;
    boolean clawTargetUp = false;
    boolean clawSpinLeft = false;
    boolean clawSpinRight = false;
    boolean shoulderTargetPosSet = false;
    boolean elbowTargetPosSet = false;
    boolean hangTimerSet = false;
    ElapsedTime hangTimer = new ElapsedTime();
    boolean hangTimer2Set = false;
    ElapsedTime hangTimer2 = new ElapsedTime();
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
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, -Math.PI));
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
        boolean lastGamepadB = false;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;
        boolean lastGamepad2A = false;
        boolean cutDrivePow = false;
        boolean lastG2Back = false;


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
                arm.setIntegralSumSTo0();
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

            if(gamepad2.right_bumper || gamepad1.a){
                if(!lastGamepad2A){
                    closeClaw = !closeClaw;
                }
                lastGamepad2A = true;
            } else {
                lastGamepad2A = false;
            }

            if(gamepad2.left_bumper){
                if(!lastGamepadB){
                    clawTargetUp = !clawTargetUp;
                    targetHangeClaw = false;
                }
                lastGamepadB = true;
            } else {
                lastGamepadB = false;
            }
            
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
                    dropPos = Math.abs(drive.pose.position.x) + Math.abs(drive.pose.position.y);
                }
                lastGamepadY = true;
            } else {
                lastGamepadY = false;
            }


            if(targetHangeClaw){
                arm.targetHangPos();
                closeClaw = false;
            } else {
                arm.levelClaw(clawTargetUp);
            }
            arm.closeClaw(closeClaw);
            arm.spinClaw();

            if (gamepad1.start){
                headingReset = drive.pose.heading.toDouble();
                targetAngle = drive.pose.heading.toDouble()-headingReset;
            }

            if (gamepad2.a || gamepad1.x) {
                arm.shoulderTarget(1340);
//                if (gamepad2.right_bumper || gamepad1.a) {
//                    arm.elbowTarget(-5925, .7);
//                } else {
                    //arm.elbowTarget(-5695,1);
//                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;

//            } else if (moveArmToPickupPos) {
//                arm.moveArmToPoss(12195);
//            } else if (moveArmToReadyPos) {
//                arm.moveArmToPoss(11609);
//            } else if (moveArmToDropPos) {
//                arm.moveArmToPoss(4525);
            } else if (gamepad2.x) {
                arm.shoulderTarget(3750);
                //arm.elbowTarget(-5430,1);
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
            } else if (gamepad2.y || gamepad1.y) {
                arm.shoulderTarget(2700);
                //arm.elbowTarget(-5430,1);
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
            } else if (gamepad2.dpad_up) {
                cutDrivePow = false;
                arm.shoulderTarget(2500);
                arm.elbowTarget(-4650, 1);
                targetHangeClaw = true;

                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                hangTimerSet = false;
                hangTimer2Set = false;
                hangManualy = false;
            } else if (gamepad2.dpad_left) {
                cutDrivePow = true;
                hangTimer2Set = false;
                if (!hangTimerSet){
                    hangTimer.reset();
                }
                hangTimerSet = true;
                if (hangTimer.seconds() < 3){
                    arm.shoulderTarget(50);
                    arm.elbowTarget(-4650, 1);
                } else if (hangTimer.seconds() < 7) {
                    //cutDrivePow = true;
                    arm.shoulderTarget(325);
                    arm.elbowTarget(-200, 1);
                } else if (hangTimer.seconds() < 8.2) {
                    arm.shoulderTarget(1300);
                    arm.elbowTarget(-200, 1);
                } else if (hangTimer.seconds() < 11) {
                    arm.shoulderTarget(2150);//2150
                    arm.elbowTarget(-5200, 1);
                }

                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                targetHangeClaw = true;
                hangManualy = false;
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
            } else if (gamepad2.start) {
                arm.shoulderTarget(1650);
                arm.elbowTarget(-5200, 1);
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
//            } else if (gamepad2.y) {
//                arm.moveWithStick(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
            } else {
                shoulder.setPower(0);
//                lastWasJoystick = true;
//                if (!hangManualy){
//                    if (manualControl){
//                        arm.moveShoulderManually(-gamepad2.right_stick_y);
//                        arm.moveElbowManually(-gamepad2.left_stick_y);
//                    } else {
////                       arm.calcArmPowersWithStick();
//                        arm.moveShoulder(-gamepad2.right_stick_y);
//                        arm.moveElbow(-gamepad2.left_stick_y);
//                    }
//                }
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

        public void closeClaw(boolean close){
            if (close){
                clawClose.setPosition(.45);
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
        public void levelClaw(boolean flat){
            endArmAngle = ((-shoulder.getCurrentPosition() - 1457)/8192.0 * 2*Math.PI)+ ((-elbow.getCurrentPosition() + 3344)/8192.0 * 2*Math.PI);
            servoTarget = -endArmAngle/Math.PI*.75+.11;
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

            double shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * GravityK;

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

            out = (KpE * error) + (KiE * integralSumE) + (KdE * derivative);
            if (out > maxPowUpE){
                out = maxPowUpE;
            } else if (out < - maxPowDownE){
                out = -maxPowDownE;
            }

            elbow.setPower(out);

            lastErrorE = error;

            // reset the timer for next time
            timerE.reset();

//            if (Math.abs(error) < 20) {
//                moveArmToDropPos = false;
//                moveArmToHookPoss = false;
//                moveArmToPickupPos = false;
//                moveArmToReadyPos = false;
//            }
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
        double ticksPerRev = ticksPerMotorRev * gearRatio;
        double maxVelocity = 0.125;
        double shoulderAngle = 0;

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
