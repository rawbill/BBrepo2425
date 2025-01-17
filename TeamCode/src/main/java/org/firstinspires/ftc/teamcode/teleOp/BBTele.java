package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


@TeleOp(name = "OP Tele :)")
public class BBTele extends LinearOpMode {
    Drivetrain drive;
    Slides slides;
    org.firstinspires.ftc.teamcode.subsystems.IO IO;
    public void runOpMode() {
        drive = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        IO = new IO(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("BOOOO");
            drive.updateCtrls(gamepad1, gamepad2);
            slides.updateCtrls(gamepad1, gamepad2);
            IO.updateCtrls(gamepad1, gamepad2);
            telemetry.update();
        }
    }
}
