package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;

@Config
public class IO implements Subsystem {
    private Servo leftGb;
    private Servo rightGb;
    private Servo clawPiv;
    private Servo clawRot;
    private Servo claw;
    Telemetry telemetry;

    private Timer timer = new Timer();

    public double gbPos, pivPos, rotPos = 0.5, clawPos, clawOpen = 0.65, clawClose = 0.25;

    public boolean dPad = false, ddToggle = false, rBump = false, rbToggle = false, rPad = false, lPad = false, inv = false;

    public double invOffset = 0.025;



    public IO(HardwareMap hardwareMap, Telemetry telemetry) {
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
        gbPos = 1;
        pivPos = 0.9;
        this.clawClose();
    }

    public void rest() {
        gbPos = 0.45;
        pivPos = 0.2;
        rotPos = 0.5;
    }

    public void straight() {
        gbPos = 0.55;
        pivPos = 0.6;
        rotPos = 0.5;
    }
    public void upSub() {
        gbPos = 0.45;
        pivPos = 1;
        rotPos = 0.5;
    }

    public void spec4tele() {
        gbPos = 0.8;
        pivPos = 0.7;
        rotPos = 0.5;
    }

    public void spec4auto() {
        gbPos = 0.8;
        pivPos = 0.65;
        rotPos = 0.5;
    }


    public void clawOpen()  {clawPos = clawOpen;}
    public void clawClose() {clawPos = clawClose;}

//    public void intakeInit() {
//        straight();
//        clawOpen();
//        timer.resetTimer();
//    }
    public void intakeInit() {
        upSub();
//        clawOpen();
        timer.resetTimer();
    }

    public void intake(Gamepad gp2, double ext) {
        rotate(gp2);

        if (inv) {
            gbPos = gbSetter(ext, invOffset);
            pivPos = pivSetter(ext, invOffset);
        }

        if (gp2.dpad_right && !rPad) {

            invOffset += 0.025;
            rPad = true;

        } else if (!gp2.dpad_right) {
            rPad = false;
        }

        if (gp2.dpad_left && !lPad) {

            invOffset -= 0.025;
            lPad = true;

        } else if (!gp2.dpad_left) {
            lPad = false;
        }



        if (gp2.dpad_down && !dPad) {

            ddToggle = !ddToggle;
            if (ddToggle) {
                //straight();
                upSub();
                inv = false;
            } else {
                inv = true;
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
        ddToggle = false;
//        clawOpen();
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
//                gbPos = 0.75;
//                pivPos = 0.6;
//                rotPos = 0.5;
                spec4tele();
            } else {
                specimenInit();

            }
            dPad = true;
        } else if (!gp2.dpad_down) {
            dPad = false;
        }

    }

    public void rotate(Gamepad gp2) {
        if (gp2.left_trigger > 0.8)  rotPos+=0.025;
        if (gp2.right_trigger > 0.8) rotPos-=0.025;
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

    public double gbSetter(double input, double dOffset) {
        // Clip the input to the valid range [0, 1000]
        double clippedInput = Math.max(0, Math.min(input, 1000));

        // Calculate the mapped value directly
        return dOffset + 0.75 + (clippedInput - 0) * (1 - 0.75) / (1000 - 0);
    }

    public double pivSetter(double input, double dOffset) {
        // Clip the input to the valid range [0, 1000]
        double clippedInput = Math.max(0, Math.min(input, 1000));

        // Calculate the mapped value directly
        return -dOffset + 0.8 + (clippedInput - 0) * (0.625 - 0.8) / (1000 - 0);
    }
}
