package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.stateMachines.TeleOpSM;
import org.firstinspires.ftc.teamcode.subsystems.DR4B;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides_old;


@TeleOp(name="StateMachineTeleOp", group="TeleOp")
public class StateMachineTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    DR4B dr4b;
    Slides_old slides;
    Intake intake;
    Outtake outtake;

    StateMachine machine;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        dr4b = new DR4B(hardwareMap, telemetry);
        slides = new Slides_old(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);

        machine = TeleOpSM.teleOpSM(intake, outtake, gamepad2);

        waitForStart();
        runtime.reset();

        machine.start();

        while (opModeIsActive()) {

            drivetrain.updateCtrls(gamepad1, gamepad2);
            dr4b.updateCtrls(gamepad1, gamepad2);
            slides.updateCtrls(gamepad1, gamepad2);
            machine.update();

            telemetry.update();
        }
        machine.stop();
        machine.update();
    }

}
