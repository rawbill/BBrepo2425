package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {
    public void init();
    public void update();
    public void updateCtrls(Gamepad gp1, Gamepad gp2);
}