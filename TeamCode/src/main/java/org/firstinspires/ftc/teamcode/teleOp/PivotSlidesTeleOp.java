package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;


@TeleOp(name="PivotSlidesTeleOp", group="TeleOp")
public class PivotSlidesTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    EndEffector endF;

    private Timer timer;
    private int state;

    private static double pivInit, pivFront, pivUpright;
    private static double extIn, extOut;

    private boolean xPressed = false, aPressed = false, bPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        endF = new EndEffector(hardwareMap, telemetry);


        Subsystem[] subsystems = new Subsystem[] {
                drivetrain, slides, endF
        };

        for (Subsystem system : subsystems) system.init();
        setState(0);
        timer = new Timer();

        while (opModeInInit()) {
            stateMachine();
            slides.update();
            endF.update();
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            stateMachine();

            for (Subsystem system : subsystems) system.updateCtrls(gamepad1, gamepad2);
            for (Subsystem system : subsystems) system.update();

            telemetry.update();
        }
    }

    public double timer() {return timer.getElapsedTimeSeconds();}
    public void setState(int state) {this.state = state;}

    public void stateMachine() {

        final int INIT = 0, INTAKE = 1, TRANSITION = 2, OUTTAKE = 3;
        switch (state) {
            case INIT:
                slides.setPivTarget(pivInit);
                slides.setExtTarget(extIn);
                if (opModeIsActive()) {
                    setState(TRANSITION);
                    timer.resetTimer();
                }
                break;
            case INTAKE:

                if (timer() < 1) {slides.setPivTarget(pivFront);}
                if (timer() > 1 && timer() < 2) {
                    slides.setExtTarget(extOut);
                    endF.intakeInit();
                }
                if (timer() > 2) {
                    endF.intake(gamepad2);
                }

                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;

                    setState(TRANSITION);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
                break;
            case TRANSITION:

                if (timer() < 1) {slides.setExtTarget(extIn);}
                if (timer() > 1 && timer() < 2) {
                    slides.setPivTarget(pivUpright);
                    endF.outtakeInit();
                }

                // transitions
                if (gamepad2.x && !xPressed) {
                    xPressed = true;
                    setState(OUTTAKE);
                    timer.resetTimer();
                } else if (!gamepad2.x) xPressed = false;

                if (gamepad2.b && !bPressed) {
                    bPressed = true;
                    setState(INTAKE);
                    timer.resetTimer();
                } else if (!gamepad2.b) bPressed = false;
                break;
            case OUTTAKE:

                if (timer() < 1) {slides.setPivTarget(pivUpright);}
                if (timer() > 1) {
                    slides.setExtTarget(extOut);
                    endF.outtake(gamepad2);
                }

                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;

                    setState(TRANSITION);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
                break;
        }

    }


}
