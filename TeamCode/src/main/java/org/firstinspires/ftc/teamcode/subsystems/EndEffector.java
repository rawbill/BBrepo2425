package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class EndEffector implements Subsystem {
    private Servo leftGb;
    private Servo rightGb;
    private Servo clawPiv;
    private Servo clawRot;
    private Servo claw;
    Telemetry telemetry;

    private Timer timer = new Timer();

    public static double gbPos, pivPos, rotPos = 0.5, clawPos, clawOpen = 1, clawClose = 0;

    private boolean dPad = false, ddToggle = false, rBump = false, rbToggle = false, rPad = false, rdToggle = false;



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
        gbPos = 0.1;
        pivPos = 0.35;
    }

    public void rest() {
        gbPos = 0.45;
        pivPos = 0.2;
        rotPos = 0.5;
    }

    public void straight() {
        gbPos = 0.55;
        pivPos = 0.4;
        rotPos = 0.5;
    }

    public void clawOpen()  {clawPos = clawOpen;}
    public void clawClose() {clawPos = clawClose;}

    public void setGbPos(double pos) {gbPos = pos;}

    public void setRotate(double pos) {rotPos = pos;}

    public void setPivot(double pos) {pivPos = pos;}

    public void intakeInit() {
        gbPos = 0.8;
        pivPos = 0.7;
        rotPos = 0.5;
        clawOpen();
        timer.resetTimer();
    }

    public void intake(Gamepad gp2) {
        rotate(gp2);
        if (gp2.dpad_down && !dPad) {

            ddToggle = !ddToggle;
            if (ddToggle) {
                gbPos = 1;
                pivPos = 0.6;
            } else {
                gbPos = 0.8;
                pivPos = 0.7;
            }
            dPad = true;

        } else if (!gp2.dpad_down) {
            dPad = false;
        }

        if (gp2.right_bumper && !rBump) {
            rBump = true;
            rbToggle = !rbToggle;
            if (rbToggle) {
                clawClose();
            } else {
                clawOpen();
            }
        } else if (!gp2.right_bumper) {
            rBump = false;
        }

        if (gp2.dpad_right && !rPad) {
            rPad = true;
            rdToggle = !rdToggle;
            if (rdToggle) {
                straight();
                dPad = true;
            } else {
                dPad = false;
                ddToggle = true;
            }
        } else if (!gp2.dpad_right) {
            rPad = false;
        }
    }

    public void outtakeInit() {
        gbPos = 0.5;
        pivPos = 0.2;
        rotPos = 0.5;
    }

    public void outtake(Gamepad gp2) {
        rotate(gp2);
        if (gp2.right_bumper && !rBump) {
            rBump = true;
            rbToggle = !rbToggle;
            if (rbToggle) {
                clawClose();
            } else {
                clawOpen();
            }
        } else if (!gp2.right_bumper) {
            rBump = false;
        }
    }

    public void specimenInit() {
        gbPos = 0;
        pivPos = 0.075;
        clawOpen();
    }

    public void specimen(Gamepad gp2) {
        rotate(gp2);
        if (gp2.right_bumper && !rBump) {
            rBump = true;
            rbToggle = !rbToggle;
            if (rbToggle) {
                clawClose();
            } else {
                clawOpen();
            }
        } else if (!gp2.right_bumper) {
            rBump = false;
        }

        if (gp2.dpad_down && !dPad) {
            ddToggle = !ddToggle;
            if (ddToggle) {
                gbPos = 0.75;
                pivPos = 0.6;
            } else {
                straight();
//                clawOpen();
            }
            dPad = true;
        } else if (!gp2.dpad_down) {
            dPad = false;
        }

    }

    public void rotate(Gamepad gp2) {
        if (gp2.left_trigger > 0.8)  rotPos+=0.0075;
        if (gp2.right_trigger > 0.8) rotPos-=0.0075;
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
//        if (gp2.right_trigger > 0.1) clawClose();
//        else if (gp2.left_trigger > 0.1) clawOpen();
//
//        if (gp2.dpad_down) setGbPos(0);
//        else if(gp2.dpad_up) setGbPos(1);
//
//        if (gp2.right_bumper) setRotate(0);
//        else if (gp2.left_bumper) setRotate(1);
//
//        if (gp2.dpad_left) setPivot(0);
//        else if (gp2.dpad_right) setPivot(1);

//        leftGb.setPower(-gp2.right_stick_y);
//        rightGb.setPower(-gp2.right_stick_y);

    }
}
