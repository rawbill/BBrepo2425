package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.EndEffector;

@TeleOp
public class IntakeServoTesters extends LinearOpMode {
    public EndEffector io;

    public void runOpMode() {
       io = new EndEffector(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            io.intake(gamepad2);

        }
    }
}
