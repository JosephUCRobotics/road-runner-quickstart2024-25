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

@TeleOp(name = "Arm Test", group = "B")
@Config
public class ArmTest extends LinearOpMode {
    public static double KpS = 1;
    public static double KpE = 1;
    public static double KiS = 0;
    public static double KdS = 0;
    public static double KvS = 5;
    public static double KvE = 5;
    public static double KaS = 0;

    public static double targetPosMul = 0.005;
    public static double KgS = .0025;
    public static double KgE = .0025;
    PIDController shoulderController;
    private DcMotorEx shoulder;
    private DcMotorEx elbow;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //shoulderController = new PIDController(KpS, KiS, KdS, KvS, KaS);

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

        while (opModeIsActive()) {
            //shoulderController.updatePID(KpS, KiS, KdS, KvS, KaS);
//            moveShoulder();
//            moveElbow();
            calcArmPowersWithStick();

            telemetry.update();
        }
    }
    int ticksPerMotorRev = 28;
    double gearRatio = 125 * (50/14);
    double ticksPerRev = ticksPerMotorRev * gearRatio;
    double maxVelocity = 0.125;
    double shoulderAngle = 0;

    public void moveShoulder(double powIn){
        double vel = shoulder.getVelocity() / ticksPerRev;

        double pos = shoulder.getCurrentPosition() + ticksPerRev*.25;

        shoulderAngle = (pos/ticksPerRev)*(2*Math.PI);

        double shoulderCosAngle = Math.cos(shoulderAngle);

        double desiredVelocity = -powIn*maxVelocity;

        //double pow = cosAngle * (KvS*desiredVelocity + KgS) + Kv2S*desiredVelocity;
        /*if (pow == 0){
            pow = .00001;
        }*/
        double pow = KgS * shoulderCosAngle + KvS * desiredVelocity + KpS * (desiredVelocity - vel);

        //double pow = shoulderController.output(-gamepad1.left_stick_y*maxVelocity, vel,-gamepad1.left_stick_y*maxVelocity);


        //lastPos = pos;
        //lastTime = time;
        //lastPow = pow;



        //pow += angleMultiplyer;
        //pow += shoulderGravityCorection * angleMultiplyer;

        shoulder.setPower(pow);
        telemetry.addData("raw vel", shoulder.getVelocity());
        telemetry.addData("rev/sec vel", vel);
        telemetry.addData("pow", pow);
        telemetry.addData("pos", pos);
        telemetry.addData("angle(RAD)", (pos/ticksPerRev)*(2*Math.PI));
        telemetry.addData("Cos(angle)", shoulderCosAngle);
        telemetry.addData("desiredVelocity", desiredVelocity);
        telemetry.addData("","");



    }

    public void moveElbow(double powIn){
        double vel = elbow.getVelocity() / ticksPerRev;

        double pos = elbow.getCurrentPosition() + ticksPerRev;

        double cosAngle = Math.cos((pos/ticksPerRev)*(2*Math.PI)+shoulderAngle);

        double desiredVelocity = powIn*maxVelocity;

        //double pow = cosAngle * (KvS*desiredVelocity + KgS) + Kv2S*desiredVelocity;
        /*if (pow == 0){
            pow = .00001;
        }*/
        double pow = KgE * cosAngle + KvE * desiredVelocity + KpE * (desiredVelocity - vel);

        //double pow = shoulderController.output(-gamepad1.left_stick_y*maxVelocity, vel,-gamepad1.left_stick_y*maxVelocity);


        //lastPos = pos;
        //lastTime = time;
        //lastPow = pow;



        //pow += angleMultiplyer;
        //pow += shoulderGravityCorection * angleMultiplyer;

        elbow.setPower(pow);
        telemetry.addData("raw vel", elbow.getVelocity());
        telemetry.addData("rev/sec vel", vel);
        telemetry.addData("pow", pow);
        telemetry.addData("pos", pos);
        telemetry.addData("angle(RAD)", (pos/ticksPerRev)*(2*Math.PI));
        telemetry.addData("Cos(angle)", cosAngle);
        telemetry.addData("desiredVelocity", desiredVelocity);

        telemetry.update();

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

        v[0] = gamepad1.left_stick_x;
        v[1] = -gamepad1.left_stick_y;

        // check to see if the arm is in the upper half or the lower half
        if (angle[0] >= 0) {
            inUpperHalf = true;
        } else {
            inUpperHalf = false;
        }

        // Set if the arm should bend up or down
        if (gamepad2.a){
            if (!aDown2){
                bendUp = !bendUp;
                aDown2 = true;
            }
        } else {
            aDown2 = false;
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

    public double[][] invert(double a[][]) {
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
    public void gaussian(double a[][], int index[]) {
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
}
