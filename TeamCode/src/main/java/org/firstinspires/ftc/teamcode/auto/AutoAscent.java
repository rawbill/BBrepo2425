package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DR4B;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides_old;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;


@Autonomous(name="AscentAuto", group="Auto")

public class AutoAscent extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    DR4B dr4b;
    Slides_old slides;
    Intake intake;
    Outtake outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        dr4b = new DR4B(hardwareMap, telemetry);
        slides = new Slides_old(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);

        Subsystem[] subsystems = new Subsystem[] {
                drivetrain, dr4b, slides, intake, outtake
        };

        for (Subsystem system : subsystems) {
            system.init();
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            drivetrain.imu().resetYaw();
            drivetrain.startEncoder();
            slides.moveSlides(0);

            intake.close();
            drivetrain.encoderDriveRightInches(30);
            drivetrain.imuCorrection(0,0.4);

            outtake.outSamp();
            drivetrain.encoderDriveForwardInches(9.5);
            drivetrain.imuCorrection(0,0.4);
            sleep(250);
            intake.gbDown();
            intake.pivDown();
            sleep(375);
            intake.open();
            drivetrain.encoderDriveBackwardInches(2);
            drivetrain.imuCorrection(0,0.4);
            intake.pivUp();
            outtake.in();
            drivetrain.encoderDriveBackwardInches(5);
            drivetrain.imuCorrection(-90,0.4);

            drivetrain.imuCorrection(-90,0.4);

            drivetrain.encoderDriveBackwardInches(16);
            drivetrain.imuCorrection(-90,0.4);
            drivetrain.encoderDriveLeftInches(48);
            drivetrain.imuCorrection(-90,0.4);
            intake.gbUp();
            intake.pivMid();
            drivetrain.encoderDriveForwardInches(19);
            drivetrain.imuCorrection(-90,0.4);

            drivetrain.stopEncoder();
            drivetrain.imu().resetYaw();
        }
    }
}
