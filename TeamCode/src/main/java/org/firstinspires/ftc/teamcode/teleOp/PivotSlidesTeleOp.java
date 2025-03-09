package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

@Config
@TeleOp(name="PivotSlidesTeleOp", group=" ")
public class PivotSlidesTeleOp extends LinearOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Slides slides;
    IO io;

    private Timer timer;
    public static int state;

    private boolean xPressed = false, aPressed = false, bPressed = false, yPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        slides = new Slides(hardwareMap, telemetry);
        io = new IO(hardwareMap, telemetry);

        Subsystem[] subsystems = new Subsystem[] {
                drivetrain, slides, io
        };

        for (Subsystem system : subsystems) system.init();
        sleep(250);
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

            telemetry.addData("STATE", state);
            telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
            telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
            telemetry.addData("gbSetter", io.gbSetter(slides.spools()[0].getCurrentPosition(), io.invOffset));
            telemetry.addData("gbPos", io.gbPos);
            telemetry.addData("pivSetter", io.pivSetter(slides.spools()[0].getCurrentPosition(), io.invOffset));
            telemetry.addData("pivPos", io.pivPos);

            telemetry.update();
        }
    }

    public double timer() {return timer.getElapsedTimeSeconds();}
    public void setState(int state) {PivotSlidesTeleOp.state = state;}

    public void stateMachine() {

        final int INIT = 0, INTAKE = 1, REST = 2, SPECIMEN = 3, ASCEND = 4;
        
        switch (state) {
            case INIT:
                slides.setPivTarget(Slides.pivInit);
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
                    slides.setPivTarget(Slides.pivDown);
                }
                if (timer() > 0.25 && timer() < 0.5) {
                    io.intakeInit();
                }
                if (timer() > 0.5) {
                    slides.updateCtrls(gamepad1, gamepad2);
                    if (slides.spools()[0].getCurrentPosition() > Slides.intakeCap) slides.setExtPower(-0.3);
                    io.intake(gamepad2, slides.spools()[0].getCurrentPosition());
                }
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
            case REST:
                slides.updateCtrls(gamepad1, gamepad2);

                if (timer() < 0.25) {
                    io.outtakeInit();
                }
                if (timer() > 0.25) {
                    slides.setPivTarget(Slides.pivUp);
                }
                slides.updatePiv();
                io.outtake(gamepad2);

                // transitions

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
                
                if (gamepad2.x && !xPressed) {
                    xPressed = true;
                    setState(ASCEND);
                    timer.resetTimer();
                } else if (!gamepad2.x) xPressed = false;
                break;
            case SPECIMEN:
               
               if (timer.getElapsedTimeSeconds() < 0.125) {
                    io.specimenInit();
                }
                if (timer.getElapsedTimeSeconds() > 0.125) {
                    io.specimen(gamepad2);
                }

                slides.updatePiv();
                slides.updateCtrls(gamepad1, gamepad2);

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
                if (gamepad2.x && !xPressed) {
                    xPressed = true;
                    setState(ASCEND);
                    timer.resetTimer();
                } else if (!gamepad2.x) xPressed = false;
                break;
            case ASCEND:
                
                if (timer() < 0.25) {
                    io.outtakeInit();
                }
                if (timer() > 0.25) {
                    slides.setExtTarget(Slides.extAscend);
                    slides.update();
                }
                
                // transitions
                if (gamepad2.a && !aPressed) {
                    aPressed = true;
                    setState(REST);
                    timer.resetTimer();
                } else if (!gamepad2.a) {
                    aPressed = false;
                }
                break;
        }

    }


}
