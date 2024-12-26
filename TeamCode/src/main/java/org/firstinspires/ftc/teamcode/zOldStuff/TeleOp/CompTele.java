package org.firstinspires.ftc.teamcode.zOldStuff.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zOldStuff.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.zOldStuff.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.zOldStuff.Subsystems.DroneLauncher;

@TeleOp
public class CompTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        ArmSubsystem outtake = new ArmSubsystem(hardwareMap);
        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap);


        drivetrain.init();
        outtake.init();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivetrain.update(gamepad1, gamepad2);
            outtake.update(gamepad1, gamepad2);
            droneLauncher.update(gamepad1,gamepad2);
            telemetry.update();

        }
    }
}