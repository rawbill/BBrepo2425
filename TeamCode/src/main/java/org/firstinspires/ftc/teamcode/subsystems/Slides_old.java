package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.utilities.TelePIDConstants;

public class Slides_old implements Subsystem{

    TelePIDConstants pidConstants = new TelePIDConstants();
    PIDController pid = new PIDController(
            pidConstants.SLIDE_COEFF[0], // P
            pidConstants.SLIDE_COEFF[1], // I
            pidConstants.SLIDE_COEFF[2]  // D
    );

    private DcMotorEx slide;

    static final double COUNTS_PER_INCH_SLIDE = 537.7 / (1.5039370078740157 * 3.1415926535897932);
    // encoder resolution of 312 rpm yellow jacket divided by the circumference of the spool

    Telemetry telemetry;

    public Slides_old(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        slide = map.get(DcMotorEx.class, "slide");

        slide.setDirection(DcMotorEx.Direction.REVERSE);

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void init() {
        setPidPos("wayIn");

        telemetry.addData("DR4B", "Initialized");
        telemetry.update();
    }

    public void setPidPos(String location) {
        switch (location) {
            case "wayIn": pid.setSetPoint(0);
            case "wayOut": pid.setSetPoint(COUNTS_PER_INCH_SLIDE*19.2); break;
            case "twoThirds": pid.setSetPoint(COUNTS_PER_INCH_SLIDE*12.8); break;
            case "oneThird": pid.setSetPoint(COUNTS_PER_INCH_SLIDE*6.4); break;
            default: pid.setSetPoint(0);
        }
    }

    public double PID() {
        return pid.calculate(slide.getCurrentPosition());
    }

    public void moveSlides(double pow) {
        slide.setPower(pow);
    }

    public DcMotorEx motor() {
        return this.slide;
    }

    @Override
    public void update() {
        slide.setVelocity(PID());
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        slide.setPower(-gp2.left_stick_y);
        telemetry.addData("slidePower", slide.getPower());
    }
}
