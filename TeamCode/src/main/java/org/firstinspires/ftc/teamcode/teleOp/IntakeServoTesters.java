package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IO;

@TeleOp
public class IntakeServoTesters extends LinearOpMode {
    public IO io;

    public void runOpMode() {
       io = new IO(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {


        }
    }
}
