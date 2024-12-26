package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Subsystem {

    private Servo outGb;
//    private Servo outPiv;
    private Servo outRot;
    private Servo outClaw;

    Telemetry telemetry;

    boolean yPressed;
    boolean xPressed;
    boolean rbPressed;

    public Outtake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        outGb = map.get(Servo.class, "outGb");
//        outPiv  = map.get(Servo.class, "outPiv");
        outRot = map.get(Servo.class, "outRot");
        outClaw = map.get(Servo.class, "outClaw");

        outRot.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void init() {
        outGb.setPosition(1);

        telemetry.addData("Outake", "Initialized");
        telemetry.update();
    }

    public void in() {
        outGb.setPosition(1);
//        outPiv.setPosition(1);
        outRot.setPosition(0.95);
    }

    public void outSamp() {
        outGb.setPosition(0);
//        outPiv.setPosition(0);
        outRot.setPosition(0.95);
    }

    public void outClip() {
        outGb.setPosition(0);
//        outPiv.setPosition(0);
        outRot.setPosition(0);
    }

    public void open() {
        outClaw.setPosition(0.9);
    }

    public void close() {
        outClaw.setPosition(0.65);
    }

    @Override
    public void update() {

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
//        outGb.setPosition(gp2.right_trigger);
//        outPiv.setPosition(gp2.right_trigger);

//        outRot.setPosition(Range.clip(gp2.right_trigger, 0.2, 1));

        if (!gp2.y && !gp2.x) {
            outGb.setPosition(1);
            outRot.setPosition(0.2);
        }

        if (!gp2.y) {
            yPressed = false;
        }
        if (gp2.y && !yPressed) {
            outGb.setPosition(0);
            outRot.setPosition(0.84);
            yPressed = true;
        }

        if (!gp2.x) {
            xPressed = false;
        }
        if (gp2.x && !xPressed) {
            outGb.setPosition(0);
            xPressed = true;
        }


        if (!gp2.right_bumper) {
            rbPressed = false;
            outClaw.setPosition(0.2);
        }
        if (gp2.right_bumper && !rbPressed) {
            outClaw.setPosition(0);
            rbPressed = true;
        }

        telemetry.addData("outGb", outGb.getPosition());
//        telemetry.update();
    }
}
