package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DR4B;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides_old;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.utilities.TelePIDConstants;


@TeleOp(name="LinearTeleOp", group="TeleOp")
public class LinearTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    DR4B dr4b;
    Slides_old slides;
    Intake intake;
    Outtake outtake;

    boolean dUpPressed;
    boolean dUpToggle = false;

    boolean dRtPressed;
    boolean dRtToggle = false;

    boolean lbPressed;
    boolean lbToggle = false;

    boolean yPressed;
    boolean yToggle = false;

    boolean xPressed;
    boolean xToggle = false;

    boolean rbPressed;
    boolean rbToggle = false;

    boolean aPressed;
    boolean bPressed;

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

        waitForStart();
        runtime.reset();

        for (Subsystem system : subsystems) {
            system.init();
        }

        while (opModeIsActive()) {
//            for (Subsystem system : subsystems) {
//                system.updateCtrls(gamepad1, gamepad2);
//            }

            drivetrain.updateCtrls(gamepad1, gamepad2);

            runMechanisms();

            telemetry.update();
        }
    }

    public void runMechanisms() throws InterruptedException {

        slides.moveSlides(-gamepad2.right_stick_y);

        dr4b.moveDR4B(-gamepad2.left_stick_y, -gamepad2.left_stick_y);
//        if (-gamepad2.left_stick_y > 0.1) {
//            dr4b.setPidPos("high");
//        } else {
//            dr4b.setPidPos("low");
//            if (dr4b.motors()[0].getCurrentPosition() < 10 || dr4b.motors()[1].getCurrentPosition() < 10) {
//
//                dr4b.motors()[0].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                dr4b.motors()[1].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            }
//        }
//        dr4b.PID();
//        dr4b.update();

        telemetry.addData("Kp", TelePIDConstants.DR4B_Kp);
        telemetry.addData("Ki", TelePIDConstants.DR4B_Ki);
        telemetry.addData("Kd", TelePIDConstants.DR4B_Kd);
        telemetry.addData("4bL", dr4b.motors()[0].getVelocity());
        telemetry.addData("4bR", dr4b.motors()[0].getVelocity());

//        if (gamepad2.left_trigger < 0.1) {
//            intake.pivDown();
//        } else if (gamepad2.left_trigger > 0.1 && gamepad2.left_trigger < 0.8) {
//            intake.pivMid();
//        } else if (gamepad2.left_trigger < 1) {
//            intake.pivUp();
//        }


//        if (gamepad2.left_trigger > 0.1 && gamepad2.a && !gamepad2.b) {
//            TransferSM.getTmachine(intake, slides, outtake).start();
//        } else if (gamepad2.left_trigger > 0.1 && gamepad2.a && gamepad2.b) {
//            TransferSM.getTmachine(intake, slides, outtake).update();
     /*   } else */if (gamepad2.left_trigger > 0.1) {
            intake.gbDown();
//            TransferSM.getTmachine(intake, slides, outtake).reset();
        } else {
            intake.gbUp();
//            TransferSM.getTmachine(intake, slides, outtake).reset();
        }

        if (gamepad2.dpad_up && !dUpPressed) {
            dUpPressed = true;
            dUpToggle = !dUpToggle;

            if (dUpToggle) {
                intake.pivUp();
            } else {
                intake.pivDown();
            }
        } else if (!gamepad2.dpad_up) {
            dUpPressed = false;
        }

        if (gamepad2.dpad_right && !dRtPressed) {
            dRtPressed = true;
            dRtToggle = !dRtToggle;

            if (dRtToggle) {
                intake.pivMid();
            } else {
                intake.pivDown();
            }
        } else if (!gamepad2.dpad_right) {
            dRtPressed = false;
        }

        if (gamepad2.left_bumper && !lbPressed) {
            lbPressed = true;
            lbToggle = !lbToggle;
            if (lbToggle) {
                intake.open();
            } else {
                intake.close();
            }
        } else if (!gamepad2.left_bumper) {
            lbPressed = false;
        }

//        if (!gamepad2.a) {
//            aPressed = false;
//        }
//        if (gamepad2.a && !aPressed) {
//            intake.close();
//            intake.gbUp();
//            intake.pivUp();
//            intake.open();
//            outtake.close();
//            slides.moveSlides(1);
//            slides.moveSlides(0);
//            outtake.outSamp();
//            slides.moveSlides(-1);
//            slides.moveSlides(0);
//            aPressed = true;
//        }
//
//        if (!gamepad2.b) {
//            bPressed = false;
//        }
//        if (gamepad2.b && !bPressed) {
//            intake.close();
//            intake.gbUp();
//            intake.pivUp();
//            intake.open();
//            outtake.close();
//            slides.moveSlides(1);
//            slides.moveSlides(0);
//            outtake.outClip();
//            slides.moveSlides(-1);
//            slides.moveSlides(0);
//            bPressed = true;
//        }

//        outtake
        if (gamepad2.y && !yPressed) {
            yPressed = true;
            yToggle = !yToggle;

            if (yToggle) {
                //intake.pivMid();
                outtake.outClip();
            } else {
                outtake.in();
            }
        } else if (!gamepad2.y) {
            yPressed = false;
        }

        if (gamepad2.x && !xPressed) {
            xPressed = true;
            xToggle = !xToggle;

            if (xToggle) {
                //intake.pivMid();
                outtake.outSamp();
            } else {
                outtake.in();
            }
        } else if (!gamepad2.x) {
            xPressed = false;
        }

        if (gamepad2.right_bumper && !rbPressed) {
            rbPressed = true;
            rbToggle = !rbToggle;
            if (rbToggle) {
                outtake.open();
            } else {
                outtake.close();
            }
        } else if (!gamepad2.right_bumper) {
            rbPressed = false;
        }
    }

}
