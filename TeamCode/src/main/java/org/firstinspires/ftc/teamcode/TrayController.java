package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TrayController {
    private CRServo tray;
    private DcMotor intake;
    private PIDController servoController;
    int magPos = 0;
    private boolean autoSort = true;
    private String code = "PPP";
    private int codeStep = 0;
    int[] zoneColors = {0,0,0,0,0};
//    Telemetry telemetry;
    int targetColor = 5;

    TrayController(HardwareMap hardwareMap){
//        telemetry = telemetryIn;

        tray = hardwareMap.get(CRServo.class, "magazen");
        intake = hardwareMap.get(DcMotor.class, "intake");

        servoController = new PIDController(0.001,0,0.000025);

        magPos = intake.getCurrentPosition() % 2731;
        if (magPos > 1366) {
            magPos -= 2721;
        } else if (magPos < -1366) {
            magPos += 2731;
        }
        magPos = intake.getCurrentPosition()-magPos;

    }
    void update(){
//        telemetry.addData("targetColor", targetColor);
//        telemetry.addData("codeStep", codeStep);
//        telemetry.addData("Zone 1", zoneColors[0]);
//        telemetry.addData("Zone 2", zoneColors[1]);
//        telemetry.addData("Zone 3", zoneColors[2]);
//        telemetry.addData("Zone 4", zoneColors[3]);
//        telemetry.addData("Zone 5", zoneColors[4]);
//        telemetry.update();

        tray.setPower(servoController.update(magPos, intake.getCurrentPosition()));
    }
    void update(int[] zoneColorsIn){
        zoneColors = zoneColorsIn;
        if (autoSort) {
            sortWithCam();
        }

//        telemetry.addData("targetColor", targetColor);
//        telemetry.addData("codeStep", codeStep);
//        telemetry.addData("Zone 1", zoneColors[0]);
//        telemetry.addData("Zone 2", zoneColors[1]);
//        telemetry.addData("Zone 3", zoneColors[2]);
//        telemetry.addData("Zone 4", zoneColors[3]);
//        telemetry.addData("Zone 5", zoneColors[4]);
//        telemetry.update();
        tray.setPower(servoController.update(magPos, intake.getCurrentPosition()));
    }
    void setZoneColors(int[] zoneColorsIn){
        zoneColors = zoneColorsIn;
    }

    void spinLeft(){
        magPos += 2731;
        servoController.resetTimer();
        zoneColors = new int[]{zoneColors[1], zoneColors[2], zoneColors[0], zoneColors[3], zoneColors[4]};
    }
    void spinRight(){
        magPos -= 2731;
        servoController.resetTimer();
        zoneColors = new int[]{zoneColors[2], zoneColors[0], zoneColors[1], zoneColors[3], zoneColors[4]};
    }

    void setAutoSortTo(boolean trueOrFalse){
        autoSort = trueOrFalse;
    }
    void resetTrayPosition(){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magPos = 0;
    }
    void sortWithCam(){
        if (Math.abs(magPos-intake.getCurrentPosition()) < 200) {
            if (zoneColors[0] == 0 || zoneColors[1] == 0 || zoneColors[2] == 0) {
                if (zoneColors[1] == 0) {
                    if ((zoneColors[4] == 0)){
                        if (zoneColors[0] > 0){
                            spinRight();
                        } else if (zoneColors[2]  > 0) {
                            spinLeft();
                        }
                    } else {
                        if (zoneColors[2]  > 0) {
                            spinLeft();
                        } else if (zoneColors[0] > 0){
                            spinRight();
                        }
                    }
                } else if (zoneColors[3] > 0 && zoneColors[2] == 0) {
                    spinRight();
                } else if (zoneColors[4] > 0 && zoneColors[0] == 0) {
                    spinLeft();
                }
            }
        }
    }
    void setCode(String newCode) {
        code = newCode.toUpperCase();
        codeStep = 0;
    }

    void spinToNextInCode() {
        if (codeStep > 0){
            zoneColors[1] = -1;
        }

        codeStep ++;

        if (codeStep > code.length()){
            spinRight();
        } else {
//            Convert the string into a list of numbers.    "GPP" -> {2, 1, 1}
            code = code.toUpperCase();
            int[] numberCode = {0, 0, 0};
            for (int i = 0; i < code.length(); i++) {
                char character = code.charAt(i);
                if (character == 'P') {
                    numberCode[i] = 1;
                } else if (character == 'G') {
                    numberCode[i] = 2;
                }
            }

//            Count the number of each color balls in the tray
//            emptyCount = 0,  purpleCount = 1, greenCount = 2
            int[] ballCount = {0, 0, 0};

            for (int i = 0; i < 3; i++) {
                if (zoneColors[i] == -1){
                    ballCount[0]++;
                } else {
                    ballCount[zoneColors[i]]++;
                }

            }


            // if their is no more balls of the target color in the tray, change the target color
            targetColor = numberCode[codeStep - 1];
            if (ballCount[targetColor] == 0) {
                if (targetColor == 1 && ballCount[2] > 0) {
                    targetColor = 2;
                } else if (targetColor == 2 && ballCount[1] > 0) {
                    targetColor = 1;
                } else {
                    targetColor = 0;
                }
            }


            if (targetColor != 0) {
                if (zoneColors[1] != targetColor) {
                    if (zoneColors[0] == targetColor) {
                        spinRight();
                    } else {
                        spinLeft();
                    }
                }
            } else {
                if (zoneColors[1] == -1){
                    if (zoneColors[2] != -1){
                        spinLeft();
                    } else {
                        spinRight();
                    }
                }
            }

        }
    }
    int[] getZoneColors(){
        return zoneColors;
    }
}
