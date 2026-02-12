package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;


@Config
@TeleOp(name="System Time")
public class SystemTime extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            long currentTimeMillis = System.currentTimeMillis();
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                Instant instant = Instant.ofEpochMilli(currentTimeMillis);
                // Use a specific time zone, e.g., "America/Chicago" or ZoneId.systemDefault()
                ZoneId zoneId = ZoneId.of("America/Chicago");
                ZonedDateTime zonedDateTime = instant.atZone(zoneId);
                // Define the format pattern (e.g., "yyyy-MM-dd HH:mm:ss.SSS")
                DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS");

                // Format the ZonedDateTime object
                String formattedTime = zonedDateTime.format(formatter);

                telemetry.addData("Formatted time: ", formattedTime);
                telemetry.update();
            }
        }
    }
}
