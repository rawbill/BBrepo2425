package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

@Config
@TeleOp(name="PivotSlidesTeleOp", group="TeleOp")
public class PivotSlidesTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    EndEffector endF;

    private Timer timer;
    private int state;

    public static double pivInit = 250, pivDown = 1500, pivUp = 0;
    public static double extIn = 0, extMid = 1600, extOut = 2375;

    private boolean xPressed = false, aPressed = false, bPressed = false, yPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        endF = new EndEffector(hardwareMap, telemetry);


        Subsystem[] subsystems = new Subsystem[] {
                drivetrain, slides, endF
        };

        for (Subsystem system : subsystems) system.init();
        sleep(1000);
        setState(2);
        timer = new Timer();

        while (!opModeIsActive()) {
            stateMachine();
            slides.update();
            endF.update();
            telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        setState(2);

        while (opModeIsActive()) {

            stateMachine();

            drivetrain.updateCtrls(gamepad1, gamepad2);
//            endF.updateCtrls(gamepad1, gamepad2);
            endF.update();

            telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
            telemetry.addData("exrtPos ", slides.spools()[0].getCurrentPosition());

            telemetry.update();
        }
    }

    public double timer() {return timer.getElapsedTimeSeconds();}
    public void setState(int state) {this.state = state;}

    public void stateMachine() {

        final int INIT = 0, INTAKE = 1, REST = 2, OUTTAKE = 3, SPECIMEN = 4;
        switch (state) {
            case INIT:
                slides.setPivTarget(pivInit);
                slides.setExtTarget(25);
                endF.init();
                slides.updatePiv();
                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;

                    setState(REST);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
                break;
            case INTAKE:

                if (timer() < 0.5) {slides.setPivTarget(pivDown);}
                if (timer() > 0.5 && timer() < 1.5) {
//                    slides.setExtTarget(extMid);
                    endF.intakeInit();
                }
                if (timer() > 1.5) {
                    slides.setExtTarget(extMid);
                    endF.intake(gamepad2);
                }
                slides.updateCtrls(gamepad1, gamepad2);
                if (slides.spools()[0].getCurrentPosition() > 600) slides.setSlidePower(-0.1);
                slides.updatePiv();
//                slides.update();

                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;

                    setState(REST);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
                break;
            case REST:

                slides.updateCtrls(gamepad1, gamepad2);

                if (timer() < 0.5) {
                    endF.rest();
                    slides.setExtTarget(extIn);
                }
                if (timer() > 0.5 && timer() < 1.5) {
                    slides.setPivTarget(pivUp);
                }
                if (timer() > 1.5) {
                    slides.setPivTarget(pivUp);
                }
                slides.updatePiv();
                endF.outtake(gamepad2);

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

                if (gamepad2.y && !yPressed) {
                    yPressed = true;
                    setState(SPECIMEN);
                    timer.resetTimer();
                } else if (!gamepad2.y) yPressed = false;
                break;
            case OUTTAKE:

                slides.setPivTarget(pivUp);
                slides.setExtTarget(extOut);
                endF.outtakeInit();
                endF.outtake(gamepad2);

                slides.update();


                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;

                    setState(REST);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
                break;
            case SPECIMEN:

                slides.setPivTarget(pivUp);
                slides.setExtTarget(extIn);
                endF.specimen(gamepad2);

                slides.update();


                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;

                    setState(REST);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
        }

    }


}
