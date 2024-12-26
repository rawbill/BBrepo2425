package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {

    private Servo rightGb;
    private Servo leftGb;
    private Servo intPiv;
    private Servo intClaw;

    Telemetry telemetry;

    boolean dUpPressed;
    boolean dLfPressed;
    boolean dDwPressed;
    boolean lbPressed;

    double pivPos;

    public Intake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        rightGb = map.get(Servo.class, "rightGb");
        leftGb  = map.get(Servo.class, "leftGb");
        intPiv = map.get(Servo.class, "intPiv");
        intClaw = map.get(Servo.class, "intClaw");

        leftGb.setDirection(Servo.Direction.REVERSE);
        rightGb.setDirection(Servo.Direction.FORWARD);
        intPiv.setDirection(Servo.Direction.REVERSE);
    }

    public void init() {
        gbUp();
        pivUp();

        telemetry.addData("Intake", "Initialized");
        telemetry.update();
    }

    public void gbDown() {
        leftGb.setPosition(0.775);
        rightGb.setPosition(0.775);
    }

    public void gbUp() {
        leftGb.setPosition(0.2);
        rightGb.setPosition(0.2);
    }

    public void pivDown() {
        intPiv.setPosition(1);
    }

    public void pivMid() {
        intPiv.setPosition(0.85);
    }

    public void pivUp() {
        intPiv.setPosition(0.475);
    }

    public void open() {
        intClaw.setPosition(0.4);
    }

    public void close() {
        intClaw.setPosition(0.05);
    }


    @Override
    public void update() {

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
//        leftGb.setPosition(Range.clip(gp2.left_trigger, 0.475, 0.52));
//        rightGb.setPosition(Range.clip(gp2.left_trigger, 0.475, 0.52));
//        intPiv.setPosition(1 - Range.clip(gp2.left_trigger, 0.43, 0.57));

        if (gp2.left_trigger < 0.1) {
            pivPos = 0.57;
        } else if (gp2.left_trigger > 0.1 && gp2.left_trigger < 0.8) {
            pivPos = 0.48;
        } else if (gp2.left_trigger < 1) {
            pivPos = 0.43;
        }
        intPiv.setPosition(pivPos);

//        if (!gp2.dpad_up && !gp2.dpad_left && !gp2.dpad_down) {
//            leftGb.setPosition(0.18);
//            rightGb.setPosition(0.18);
//        }

        if (!gp2.dpad_up) {
            leftGb.setPosition(0.18);
            rightGb.setPosition(0.18);
            dUpPressed = false;
        }
        if (gp2.dpad_up && !dUpPressed) {
            leftGb.setPosition(0.725);
            rightGb.setPosition(0.725);
            dUpPressed = true;
        }

        if (!gp2.left_bumper) {
            lbPressed = false;
            intClaw.setPosition(0.2);
        }
        if (gp2.left_bumper && !lbPressed) {
            intClaw.setPosition(0);
            lbPressed = true;
        }

        telemetry.addData("leftGb", leftGb.getPosition());
        telemetry.addData("rightGb", rightGb.getPosition());
        telemetry.addData("intPiv", intPiv.getPosition());
//        telemetry.update();
    }


}
