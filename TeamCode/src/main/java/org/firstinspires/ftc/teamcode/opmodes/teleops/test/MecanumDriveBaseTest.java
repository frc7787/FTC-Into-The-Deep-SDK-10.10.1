package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

@TeleOp
public class MecanumDriveBaseTest extends OpMode {
    private MecanumDrive driveBase;
    private ElapsedTime elapsedTime;

    private ArrayList<String> timeAndVelocityPairs;

    private boolean fileSaved, initialized;

    @Override public void init() {
        driveBase = new MecanumDrive.Builder(hardwareMap)
                .build();
        elapsedTime = new ElapsedTime();
        timeAndVelocityPairs = new ArrayList<>();
        fileSaved = false;
        initialized = false;
    }

    @Override public void start() {
       elapsedTime.reset();
    }

    @Override public void loop() {
        double velocity = driveBase.localizer.par.getPositionAndVelocity().velocity;
        double time = elapsedTime.seconds();

        if (!initialized) {
            initialized = true;
            driveBase.drive(1.0, 0, 0);
        }

        if (elapsedTime.seconds() > 1.5) {
            driveBase.drive(0,0,0);
            if (!fileSaved && velocity == 0) {
                fileSaved = true;
                saveFile();
            }
        }

        timeAndVelocityPairs.add(velocity + "," + time);
    }

    private void saveFile() {
        String currentDate =
                new SimpleDateFormat("yyyyMMdd", Locale.CANADA).format(new Date());
        String currentTime =
                new SimpleDateFormat("HHmmss", Locale.CANADA).format(new Date());

        String aprilTagLogFileName = "MecanumVelocityLog" + currentDate + "_" + currentTime + ".txt";

        String pathToAprilTagLogFile
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/" + aprilTagLogFileName;

        try {
            File aprilTagLogFile = new File(pathToAprilTagLogFile);
            FileWriter fileWriter = new FileWriter(aprilTagLogFile, true);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            bufferedWriter.write("Ticks / Second,Seconds\n");

            for (String timeAndVelocityPair : timeAndVelocityPairs) {
                bufferedWriter.write(timeAndVelocityPair + "\n");
            }

            bufferedWriter.close();
        } catch (IOException e) {
            telemetry.addData("Failed to write to file", e);
        }
    }

}