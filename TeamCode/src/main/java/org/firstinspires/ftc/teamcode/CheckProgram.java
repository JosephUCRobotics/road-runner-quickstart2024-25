package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="Check Program", group = "B")
public class CheckProgram extends LinearOpMode {
    private DcMotor shoot_l;
    private DcMotor arm;
    private DcMotor back_l;
    private DcMotor front_r;
    private DcMotor back_r;
    private DcMotor intake;
    private DcMotor front_l;
    private CRServo tray;

    @Override
    public void runOpMode() {
        shoot_l = hardwareMap.get(DcMotor.class, "shoot_l");
        arm = hardwareMap.get(DcMotor.class, "arm");
        back_l = hardwareMap.get(DcMotor.class, "back_l");
        front_r = hardwareMap.get(DcMotor.class, "front_r");
        back_r = hardwareMap.get(DcMotor.class, "back_r");
        intake = hardwareMap.get(DcMotor.class, "intake");
        front_l = hardwareMap.get(DcMotor.class, "front_l");
        tray = hardwareMap.get(CRServo.class, "magazen");

        shoot_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoot_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            shoot_l.setPower(1);
            arm.setPower(1);
            intake.setPower(.2);
            front_l.setPower(1);
            back_l.setPower(1);
            front_r.setPower(1);
            back_r.setPower(1);
            tray.setPower(1);

            shoot_l.setPower(0);
            arm.setPower(0);
            front_l.setPower(0);
            back_l.setPower(0);
            front_r.setPower(0);
            back_r.setPower(0);
            tray.setPower(0);
            // Put run blocks here.
            while (opModeIsActive()) {


                if (shoot_l.getCurrentPosition() != 0 && shoot_l.getCurrentPosition() != 0 &&
                        shoot_l.getCurrentPosition() != 0 && shoot_l.getCurrentPosition() != 0 &&
                        shoot_l.getCurrentPosition() != 0 && shoot_l.getCurrentPosition() != 0 &&
                        shoot_l.getCurrentPosition() != 0) {
                    telemetry.addData("You Are Good To Go!!", "Hope Nothing Goes Wrong!!");
                    telemetry.addData("", "");
                }
                telemetry.addData("Shooter      ", shoot_l.getCurrentPosition());
                telemetry.addData("Arm          ", arm.getCurrentPosition());
                telemetry.addData("Intake       ", intake.getCurrentPosition());
                telemetry.addData("Front Left   ", front_l.getCurrentPosition());
                telemetry.addData("Front Right  ", front_r.getCurrentPosition());
                telemetry.addData("Back Left    ", back_l.getCurrentPosition());
                telemetry.addData("Back Right   ", back_r.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}