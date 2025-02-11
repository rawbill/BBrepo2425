package org.firstinspires.ftc.teamcode.automac;

import android.os.Environment;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoMac {
    private List<LogEntry> logEntries;
    private long startTime;
    private long prevTime;
    private boolean logging = false;
    private Map<String, DcMotor> motors;
    private boolean hasMotors = false;
    private Map<String, Servo> servos;
    private boolean hasServos = false;
    private Map<String, CRServo> crServos;
    private boolean hasCRServos = false;
    public static final String BASE_FOLDER_NAME = "FIRST";
    public String fullFileName;
    public static final String fileExtension = ".json";
    public String directoryPath;
    private long interval = 100;

    public AutoMac(String filename, HardwareMap hardwareMap) {
        logEntries = new ArrayList<>();
        directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        fullFileName = filename + fileExtension;
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void start() {
        startTime = System.currentTimeMillis();
        prevTime = startTime;
        logging = true;
    }

    public void setInterval(int milliSeconds) {
        this.interval = milliSeconds;
    }
    public int getInterval() {
        return (int) interval;
    }

    public void logMotors(Map<String, DcMotor> motors) {
        this.motors = motors;
        hasMotors = true;
    }
    public void logServos(Map<String, Servo> servos) {
        this.servos = servos;
        hasServos = true;
    }
    public void logCRServos(Map<String, CRServo> crServos) {
        this.crServos = crServos;
        hasCRServos = true;
    }

    public void update() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - prevTime < interval) {
            return;
        }
        LogEntry entry = new LogEntry();
        entry.setTimestamp(currentTime - startTime);
        if (hasMotors) {
            Map<String, Double> motorMap = new HashMap<>();
            motors.forEach((name, motor) -> motorMap.put(name, motor.getPower()));
            entry.setMotors(motorMap);
        }
        if (hasServos) {
            Map<String, Double> servoMap = new HashMap<>();
            servos.forEach((name, servo) -> servoMap.put(name, servo.getPosition()));
            entry.setServos(servoMap);
        }
        if (hasCRServos) {
            Map<String, Double> crServoMap = new HashMap<>();
            crServos.forEach((name, crServo) -> crServoMap.put(name, crServo.getPower()));
            entry.setCRServos(crServoMap);
        }
        prevTime = currentTime;
        logEntries.add(entry);
    }

    public void save() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        logEntries.remove(logEntries.size()-1);
        mapper.writerWithDefaultPrettyPrinter().writeValue(new File(directoryPath+"/"+fullFileName), logEntries);
    }
}