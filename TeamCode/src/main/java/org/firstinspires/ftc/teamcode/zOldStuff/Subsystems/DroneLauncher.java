package org.firstinspires.ftc.teamcode.zOldStuff.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher implements Subsystem {
    public Servo servo3;
    public static final double RELEASE_POSITION = -1;
    public static final double NORMAL_POSITION = 0.0;

    public DroneLauncher(HardwareMap hardwareMap) {
        servo3 = hardwareMap.get(Servo.class, "upDownServo");
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if(gamepad2.b) {
            release();
        } else if (gamepad2.x) {
            reset();
        }
    }

    public void release() {
        servo3.setPosition(RELEASE_POSITION);
    }

    public void reset() {
        servo3.setPosition(NORMAL_POSITION);
    }

    @Override
    public void addTelemetry(Telemetry t, boolean shouldPrint) {

    }
}
