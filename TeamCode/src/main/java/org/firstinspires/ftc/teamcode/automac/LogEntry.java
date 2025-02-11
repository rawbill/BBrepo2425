package org.firstinspires.ftc.teamcode.automac;

import java.util.Map;

class LogEntry {
    private long timestamp;
    private Map<String, Double> motors; // Motor name -> power
    private Map<String, Double> servos; // Servo name -> position
    private Map<String, Double> crServos; // CRServo name -> power

    public long getTimestamp() { return timestamp; }

    public void setTimestamp(long timestamp) { this.timestamp = timestamp; }


    public Map<String, Double> getMotors() { return motors; }

    public void setMotors(Map<String, Double> motors) {
        this.motors = motors;
    }

    public Map<String, Double> getServos() { return servos; }

    public void setServos(Map<String, Double> servos) {
        this.servos = servos;
    }

    public Map<String, Double> getCRServos() { return crServos; }

    public void setCRServos(Map<String, Double> crServos) {
        this.crServos = crServos;
    }

    @Override
    public String toString() {
        return "" + timestamp;
    }
}