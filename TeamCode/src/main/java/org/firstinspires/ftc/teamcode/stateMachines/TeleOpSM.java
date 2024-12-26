package org.firstinspires.ftc.teamcode.stateMachines;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class TeleOpSM {

    enum States {
        INTAKE_D,
        INTAKE_M,
        INTAKE_U,
        TRANSFER,
        OUTTAKE_O,
        OUTTAKE_I
    }

    public static StateMachine teleOpSM(Intake intake, Outtake outtake, Gamepad gp2) {
        return new StateMachineBuilder()
            .state(States.TRANSFER)
                .onEnter(() -> {
                    outtake.in();
                    intake.close();
                    sleep(375);
                    intake.gbUp();
                    intake.pivUp();
                })

                .transition(() -> gp2.dpad_up, States.INTAKE_U)
                .transition(() -> gp2.dpad_right, States.INTAKE_M)
                .transition(() -> gp2.dpad_down, States.INTAKE_D, intake::open)
                .transition(() -> gp2.triangle, States.OUTTAKE_O, () -> {
                    intake.open();
                    sleep(375);
                    intake.gbDown();
                })

                .state(States.INTAKE_D)
                .onEnter(() -> {
                    intake.gbDown();
                    intake.pivDown();
                })
                .onExit(() -> {
                    intake.close();
                    sleep(375);
                })
                .transition(() -> gp2.dpad_up, States.INTAKE_U)
                .transition(() -> gp2.dpad_right, States.INTAKE_M)
                .transition(() -> gp2.cross, States.TRANSFER)

                .state(States.INTAKE_M)
                .onEnter(() -> {
                    intake.gbDown();
                    intake.pivMid();
                })
                .onExit(() -> {
                    System.out.println("Exiting");
                })
                .transition(() -> gp2.dpad_up, States.INTAKE_U)
                .transition(() -> gp2.dpad_down, States.INTAKE_D, intake::open)
                .transition(() -> gp2.cross, States.TRANSFER)

                .state(States.INTAKE_U)
                .onEnter(() -> {
                    intake.gbDown();
                    intake.pivUp();
                })
                .onExit(() -> {
                    System.out.println("Exiting");
                })
                .transition(() -> gp2.dpad_right, States.INTAKE_M)
                .transition(() -> gp2.dpad_down, States.INTAKE_D, intake::open)
                .transition(() -> gp2.cross, States.TRANSFER)

                .state(States.OUTTAKE_O)
                .onEnter(() -> {
                    outtake.close();
                    sleep(375);
                    outtake.outSamp();
                    intake.pivMid();
                })
                .onExit(() -> {
                    outtake.open();
                    sleep(375);
                    outtake.in();
                })
                .transition(() -> gp2.square, States.OUTTAKE_I)

                .state(States.OUTTAKE_I)
                .onEnter(() -> {
                    outtake.open();
                    outtake.in();
                })
                .onExit(() -> {
                    intake.open();
                })
                .transition(() -> gp2.cross, States.TRANSFER)
                .transition(() -> gp2.dpad_up, States.INTAKE_U)
                .transition(() -> gp2.dpad_right, States.INTAKE_M)
                .transition(() -> gp2.dpad_down, States.INTAKE_D)

                .build();
    }
}
