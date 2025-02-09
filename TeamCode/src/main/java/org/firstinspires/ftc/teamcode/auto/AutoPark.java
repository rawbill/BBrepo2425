package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


@Autonomous(name="ParkAuto", group="Auto")

public class AutoPark extends LinearOpMode {


    private Timer specTimer;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    IO io;

    private final int STILL = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4;

    public static double pivInit = 600, pivDown = 1650, pivUp = 0, pivSpec = 400;
    public static double extIn = 0, extMid = 600, extOut = 2500;

    public double extSpec = 1500;

    public boolean bool = false;


    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        io = new IO(hardwareMap, telemetry);

        specTimer = new Timer();

        boolean ran = false;

        slides.setPivTarget(pivInit);
        slides.setExtTarget(extIn);

        io.init();
        io.clawClose();
        drivetrain.imu().resetYaw();

        // Wait for the game to start (driver presses START)
        while (!isStarted()) {
            slides.update();
            io.update();
        }
        runtime.reset();

        slides.setPivTarget(pivUp);
        io.spec4auto();
        drivetrain.stopEncoder();
        specTimer.resetTimer();
        // run until the end of the match (driver presses STOP)
        waitForStart();


        if (opModeIsActive()) {

            drivetrain.setState(LEFT, 10);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.update();
                io.update();
            }

            drivetrain.setState(FORWARD, 30);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.update();
                io.update();
            }

            while (!score()) {
                score();
                slides.update();
                io.update();
            }

            drivetrain.setState(BACKWARD, 25);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.update();
                io.update();
            }

            drivetrain.setState(RIGHT, 30);
            while (!drivetrain.stateMachine()) {
                drivetrain.stateMachine();
                slides.update();
                io.update();
            }


        }
    }

    public boolean score() {
        if (!bool) {
            slides.setPivTarget(pivUp);
            io.spec4auto();
            specTimer.resetTimer();

            bool = true;
        }

        if (specTimer() > 1.5 && specTimer() < 2.5) {
            slides.setExtTarget(extSpec);

        }

        if (specTimer() > 2.5) {
            slides.setExtTarget(extIn);
            io.clawOpen();
            bool = false;
            return true;
        } else return false;
    }

    public double specTimer() {
        return specTimer.getElapsedTimeSeconds();
    }
}