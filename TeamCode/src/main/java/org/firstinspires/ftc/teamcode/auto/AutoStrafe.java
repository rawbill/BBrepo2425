package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DR4B;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Slides_old;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;


@Autonomous(name="StrafeAuto", group="Auto")

public class AutoStrafe extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    EndEffector io;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        io = new EndEffector(hardwareMap, telemetry);

        boolean ran = false;

        io.init();
        drivetrain.imu().resetYaw();

        // Wait for the game to start (driver presses START)
        while (!isStarted()) {
            slides.setPivTarget(650);
            slides.updatePiv();
            io.update();
        }
        runtime.reset();

        slides.setPivTarget(0);
        io.straight();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!ran && slides.pivMotor().getCurrentPosition() < 10) {

                drivetrain.startEncoder();

                io.straight();

                drivetrain.encoderDriveRightInches(30);
                drivetrain.imuCorrection(0, 0.4);

                drivetrain.stopEncoder();
                io.straight();
                ran = true;
            }
            slides.updatePiv();
            io.update();
        }
    }
}
