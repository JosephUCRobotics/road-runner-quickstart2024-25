package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@Config
@TeleOp(name="Shooter Tuner")
public class shooterTuner extends LinearOpMode {
    private DcMotor shoot_top;
    private DcMotor shoot_main;

    private CRServo tray;
    private DcMotor intake;

    double tray_target = 0;

    public static double shooterTuner_topShootVelocity = 1400;
    public static double shooterTuner_mainShootVelocity = 1300;
    public static double shooterTuner_topP = 0.0015;
    public static double shooterTuner_topI = 0;
    public static double shooterTuner_topD = 0.000027;
    public static double shooterTuner_mainP = 0.006;
    public static double shooterTuner_mainI = 0;
    public static double shooterTuner_mainD = 0.00005;
    public static double shooterTuner_trayP = 0.0003;
    public static double shooterTuner_trayI = 0;
    public static double shooterTuner_trayD = 0.0000082;
    public static double shooterTuner_trayMaxPow = .2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        shoot_top = hardwareMap.get(DcMotor.class, "shoot_l");
        shoot_main = hardwareMap.get(DcMotor.class, "shoot_main");
        tray = hardwareMap.get(CRServo.class, "magazen");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Put initialization blocks here.
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_main.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot_main.setDirection(DcMotor.Direction.REVERSE);

        PIDController servoController = new PIDController(shooterTuner_trayP,shooterTuner_trayI,shooterTuner_trayD);
        PIDController topShootController = new PIDController(shooterTuner_topP, shooterTuner_topI, shooterTuner_topD);
        PIDController mainShootController = new PIDController(shooterTuner_mainP, shooterTuner_mainI, shooterTuner_mainD);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                servoController.setPIDConstants(shooterTuner_trayP,shooterTuner_trayI,shooterTuner_trayD);
                topShootController.setPIDConstants(shooterTuner_topP, shooterTuner_topI, shooterTuner_topD);
                mainShootController.setPIDConstants(shooterTuner_mainP, shooterTuner_mainI, shooterTuner_mainD);

                // Calculate the shooters power
//                double shootersVelocity = 7 * distanceFromGoal + 1300;

//                double shootersPower = shootersVelocity/2650+shootController.update(shootersVelocity, ((DcMotorEx) shoot_l).getVelocity());
                double topShootPower = shooterTuner_topShootVelocity/2650+topShootController.update(shooterTuner_topShootVelocity, ((DcMotorEx) shoot_top).getVelocity());
                double mainShootPower = shooterTuner_mainShootVelocity/2650+mainShootController.update(shooterTuner_mainShootVelocity, ((DcMotorEx) shoot_main).getVelocity());



                if (gamepad1.a) {
                    shoot_top.setPower(topShootPower);
                } else {
                    shoot_top.setPower(0);
                    topShootController.resetTimer();
                }

                if (gamepad1.b) {
                    shoot_main.setPower(mainShootPower);
                } else {
                    shoot_main.setPower(0);
                    mainShootController.resetTimer();
                }

                if (gamepad1.leftBumperWasPressed()) {
                    tray_target += 2731;
                    servoController.resetTimer();
                } else if (gamepad1.rightBumperWasPressed()) {
                    tray_target += 2731;
                    servoController.resetTimer();
                }
                double tray_pow = servoController.update(tray_target, intake.getCurrentPosition());
                if (tray_pow > shooterTuner_trayMaxPow){
                    tray_pow = shooterTuner_trayMaxPow;
                }
                tray.setPower(tray_pow);

                telemetry.addData("topShootTargetVel", shooterTuner_topShootVelocity);
                telemetry.addData("topShootCurrentVel", ((DcMotorEx) shoot_top).getVelocity());
                telemetry.addData("", "");
                telemetry.addData("mainShootTargetVel", shooterTuner_mainShootVelocity);
                telemetry.addData("mainShootCurrentVel", ((DcMotorEx) shoot_main).getVelocity());
                telemetry.addData("", "");
                telemetry.addData("trayTargetPos", tray_target);
                telemetry.addData("trayCurrentPos", intake.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}