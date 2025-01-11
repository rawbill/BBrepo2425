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


@Autonomous(name="ParkAuto", group="Auto")

public class AutoPark extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    EndEffector io;

    private final int STILL = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4;


    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        io = new EndEffector(hardwareMap, telemetry);

        boolean ran = false;

        io.init();
        io.clawClose();
        drivetrain.imu().resetYaw();

        // Wait for the game to start (driver presses START)
        while (!isStarted()) {
            slides.setPivTarget(650);
            slides.updatePiv();
            io.update();
        }
        runtime.reset();

        slides.setPivTarget(0);
        io.spec4auto();
        drivetrain.stopEncoder();
        // run until the end of the match (driver presses STOP)
        waitForStart();


        if (opModeIsActive()) {

            drivetrain.setState(FORWARD, 30);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.updatePiv();
                io.update();
            }

            io.straight();

            drivetrain.setState(BACKWARD, 10);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.updatePiv();
                io.update();
            }

            io.clawOpen();
            io.update();

            drivetrain.setState(BACKWARD, 20);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.updatePiv();
                io.update();
            }

            drivetrain.setState(RIGHT, 30);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.updatePiv();
                io.update();
            }


        }
    }
}
