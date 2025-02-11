package org.firstinspires.ftc.teamcode.automac;

import static org.firstinspires.ftc.teamcode.automac.AutoMac.BASE_FOLDER_NAME;
import static org.firstinspires.ftc.teamcode.automac.AutoMac.fileExtension;

import android.os.Environment;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

public class Replay {
    private static List<LogEntry> loadLog(String filename) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        String fullFileName = filename + fileExtension;
        String fullname = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME + "/" + fullFileName;
        return mapper.readValue(new File(fullname), mapper.getTypeFactory().constructCollectionType(List.class, LogEntry.class));
    }

    public static void replayAutoMac(String filename, Map<String, DcMotor> motors, Map<String, Servo> servos, Map<String, CRServo> crServos, Telemetry t) throws IOException, InterruptedException {
        replayMacro(loadLog(filename), motors, servos, crServos, t);
    }

    private static void replayMacro(
            List<LogEntry> logEntries,
            Map<String, DcMotor> motors,
            Map<String, Servo> servos,
            Map<String, CRServo> crServos,
            Telemetry t
    ) throws InterruptedException, IOException {
        long startTime = System.currentTimeMillis();

        for (LogEntry entry : logEntries) {
            long targetTime = entry.getTimestamp();
            t.addData("timestamp", targetTime);

            while (System.currentTimeMillis() - startTime < targetTime) {
                Thread.sleep(1);
            }

            // Apply motor powers
            if (motors != null && !motors.isEmpty()) {
                Map<String, Double> motorStates = entry.getMotors();
                for (Map.Entry<String, Double> motorState : motorStates.entrySet()) {
                    String motorName = motorState.getKey();
                    double power = motorState.getValue();
                    if (motors.containsKey(motorName)) {
                        motors.get(motorName).setPower(power);
                        t.addData(motorName, power);
                    }
                }
            }

            // Apply servo positions
            if (servos != null && !servos.isEmpty()) {
                Map<String, Double> servoStates = entry.getServos();
                for (Map.Entry<String, Double> servoState : servoStates.entrySet()) {
                    String servoName = servoState.getKey();
                    double position = servoState.getValue();
                    if (servos.containsKey(servoName)) {
                        servos.get(servoName).setPosition(position);
                    }
                }
            }

            // Apply CRServo Powers
            if (crServos != null && !crServos.isEmpty()) {
                Map<String, Double> crServoStates = entry.getCRServos();
                for (Map.Entry<String, Double> crServoState : crServoStates.entrySet()) {
                    String servoName = crServoState.getKey();
                    double power = crServoState.getValue();
                    if (crServos.containsKey(servoName)) {
                        crServos.get(servoName).setPower(power);
                    }
                }
            }
            t.update();
        }
    }
}