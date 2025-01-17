package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

@Config
@TeleOp(name="PivotSlidesTeleOp", group="TeleOp")
public class PivotSlidesTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    IO io;

    private Timer timer;
    private int state;

    public static double pivInit = 650, pivDown = 1500, pivUp = 0;
    public static double extIn = 0, extMid = 600, extOut = 2500;

    private boolean xPressed = false, aPressed = false, bPressed = false, yPressed = false, rBump = false, dPad = false, ddToggle = false, specCtrl = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        io = new IO(hardwareMap, telemetry);


        Subsystem[] subsystems = new Subsystem[] {
                drivetrain, slides, io
        };

        for (Subsystem system : subsystems) system.init();
        sleep(1000);
        setState(0);
        timer = new Timer();

        while (!opModeIsActive()) {
            stateMachine();
            io.clawClose();
            slides.updatePiv();
            io.update();
            telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
            telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        setState(2);

        while (opModeIsActive()) {

            stateMachine();

            drivetrain.updateCtrls(gamepad1, gamepad2);
            slides.updatePiv();
            io.update();

            telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
            telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
            telemetry.addData("gbSetter", io.gbSetter(slides.spools()[0].getCurrentPosition()));
            telemetry.addData("gbPos", io.gbPos);
            telemetry.addData("pivSetter", io.pivSetter(slides.spools()[0].getCurrentPosition()));
            telemetry.addData("pivPos", io.pivPos);

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
//                slides.setExtTarget(extIn);
                io.init();
                io.clawClose();
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

                if (timer() < 0.25) {
                    slides.setPivTarget(pivDown);
                }
                if (timer() > 0.25 && timer() < 0.5) {
                    slides.setExtTarget(extMid);
                    io.intakeInit();
                    slides.update();
                }
                if (timer() > 0.5) {
                    slides.updateCtrls(gamepad1, gamepad2);
                    if (slides.spools()[0].getCurrentPosition() > 600) slides.setSlidePower(-0.3);
                    io.intake(gamepad2, slides.spools()[0].getCurrentPosition());
                    slides.updatePiv();
                }
//                slides.updatePiv();
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

                if (timer() < 0.25) {
                    io.rest();
//                    slides.setExtTarget(extIn);
//                    slides.update();
                }
                if (timer() > 0.25 && timer() < 1) {
                    slides.setPivTarget(pivUp);
                }
                if (timer() > 1) {
                    slides.setPivTarget(pivUp);
                }
                slides.updatePiv();
                io.outtake(gamepad2);

                // transitions

                if (gamepad2.b && !bPressed) {
                    bPressed = true;
                    setState(INTAKE);
                    timer.resetTimer();
                } else if (!gamepad2.b) bPressed = false;

//                if (gamepad2.x && !xPressed) {
//                    xPressed = true;
//                    setState(OUTTAKE);
//                    timer.resetTimer();
//                } else if (!gamepad2.x) xPressed = false;

                if (gamepad2.y && !yPressed) {
                    yPressed = true;
                    setState(SPECIMEN);
                    timer.resetTimer();
                } else if (!gamepad2.y) yPressed = false;
                break;
            case OUTTAKE:

                slides.setPivTarget(pivUp);
                slides.setExtTarget(extOut);
                io.outtake(gamepad2);

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


//                slides.setExtTarget(extIn);
                if (timer.getElapsedTimeSeconds() < 0.5) {
                    io.clawOpen();
                    dPad = false;
                    ddToggle = false;
                    rBump = false;
                }
                if (timer.getElapsedTimeSeconds() > 0.5 && timer.getElapsedTimeSeconds() < 0.75) {
                    io.specimenInit();
                }
                if (gamepad2.dpad_down && !dPad) {
                    ddToggle = !ddToggle;
                    if (ddToggle) {
                        io.straight();
                        slides.setPivTarget(350);
                    } else {
                        slides.setPivTarget(pivUp);
                        io.specimenInit();

                    }
                    dPad = true;
                } else if (!gamepad2.dpad_down) {
                    dPad = false;
                }

                if (gamepad2.right_bumper && !rBump) {
                    rBump = true;
                    io.clawClose();

                    specCtrl = true;
                } else if (!gamepad2.right_bumper) {
                    rBump = false;
                }
                if (specCtrl) {
                    io.specimen(gamepad2);
                    rBump = true;
                }

                slides.updatePiv();
                slides.updateCtrls(gamepad1, gamepad2);
                if (slides.spools()[0].getCurrentPosition() > 2000) slides.setSlidePower(-0.2);


                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;
                    setState(REST);
                    timer.resetTimer();
                } else if (!gamepad2.a) aPressed = false;

                if (gamepad2.b && !bPressed) {
                    bPressed = true;
                    setState(INTAKE);
                    timer.resetTimer();
                } else if (!gamepad2.b) bPressed = false;
        }

    }


}
