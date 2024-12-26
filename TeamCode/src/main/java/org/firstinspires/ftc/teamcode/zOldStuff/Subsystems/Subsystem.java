package org.firstinspires.ftc.teamcode.zOldStuff.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    public void init();
    public void update();
    public void update(Gamepad gamepad1, Gamepad gamepad2);
    public void addTelemetry(Telemetry t, boolean shouldPrint);
}

