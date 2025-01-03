package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class EndEffector implements Subsystem {
    private Servo leftGb;
    private Servo rightGb;
    private Servo clawPiv;
    private Servo clawRot;
    private Servo claw;
    Telemetry telemetry;

    private Timer timer = new Timer();

    private static double gbPos, pivPos, rotPos, clawPos, clawOpen = 0, clawClose = 1;

    private boolean rBump = true, lBump = true;



    public EndEffector(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftGb = hardwareMap.get(Servo.class,"leftGb");
        rightGb = hardwareMap.get(Servo.class,"rightGb");
        clawPiv = hardwareMap.get(Servo.class,"clawPiv");
        clawRot = hardwareMap.get(Servo.class,"clawRot");
        claw = hardwareMap.get(Servo.class,"claw");

        leftGb.setDirection(Servo.Direction.REVERSE);
        rightGb.setDirection(Servo.Direction.FORWARD);
        clawPiv.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void init() {
        telemetry.addData("End Effector","Initialized");
        telemetry.update();
    }

    public void clawOpen()  {clawPos = clawOpen;}
    public void clawClose() {clawPos = clawOpen;}

    public void setGbPos(double pos) {
        leftGb.setPosition(pos);
        rightGb.setPosition(pos);
    }

    public void setRotate(double pos) {
        clawRot.setPosition(pos);
    }

    public void setPivot(double pos) {
        clawPiv.setPosition(pos);
    }

    public void intakeInit() {
        gbPos = 0.5;
        pivPos = 1;
        rotPos = 0.5;
        timer.resetTimer();
    }

    public void intake(Gamepad gp2) {
        rotate(gp2);
        if (!gp2.right_bumper && rBump) {

            if (timer.getElapsedTimeSeconds() < 0.25) {
                gbPos = 0.75;
                pivPos = 0.75;
            }
            if (timer.getElapsedTimeSeconds() > 0.25 && timer.getElapsedTimeSeconds() < 0.5) {
                clawClose();
            }
            if (timer.getElapsedTimeSeconds() > 0.5) {
                intakeInit();
                rBump = false;
            }

        } else if (gp2.right_bumper) {
            rBump = true;
        }

        if (!gp2.left_bumper && lBump) {
            lBump = false;

            clawOpen();

        } else if (gp2.left_bumper) {
            lBump = true;
        }
    }

    public void outtakeInit() {
        gbPos = 0;
        pivPos = 0;
        rotPos = 0.5;
    }

    public void outtake(Gamepad gp2) {

        if (!gp2.left_bumper && lBump) {
            lBump = false;

            clawOpen();

        } else if (gp2.left_bumper) {
            lBump = true;
        }
    }

    public void rotate(Gamepad gp2) {
        if (gp2.left_trigger > 0.1)  rotPos++;
        if (gp2.right_trigger > 0.1) rotPos--;
        clawRot.setPosition(rotPos);
    }

    @Override
    public void update() {
        leftGb.setPosition(gbPos);
        rightGb.setPosition(gbPos);
        clawPiv.setPosition(pivPos);
        clawRot.setPosition(rotPos);
        claw.setPosition(clawPos);
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp2.right_trigger > 0.1) clawClose();
        else if (gp2.left_trigger > 0.1) clawOpen();

        if (gp2.dpad_down) setGbPos(0);
        else if(gp2.dpad_up) setGbPos(1);

        if (gp2.right_bumper) setRotate(0);
        else if (gp2.left_bumper) setRotate(1);

        if (gp2.dpad_left) setPivot(0);
        else if (gp2.dpad_right) setPivot(1);

    }
}
