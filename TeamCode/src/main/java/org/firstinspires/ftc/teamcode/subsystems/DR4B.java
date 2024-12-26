package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.utilities.TelePIDConstants;

public class DR4B implements Subsystem {

    PIDController pid = new PIDController(
            TelePIDConstants.DR4B_Kp, // P
            TelePIDConstants.DR4B_Ki, // I
            TelePIDConstants.DR4B_Kd  // D
    );

    private DcMotorEx DR4Bleft;
    private DcMotorEx DR4Bright;

    double COUNTS_PER_DEGREE = 2786.2/360;
    // encoder resolution or 84 rpm yellow jacket divided by 360
    //       4 bar movement range is 85-150 degrees, or 0-65 degrees

    double lPow;
    double rPow;

    Telemetry telemetry;

    public DR4B(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        DR4Bleft = map.get(DcMotorEx.class, "4bL");
        DR4Bright = map.get(DcMotorEx.class, "4bR");


        DR4Bleft.setDirection(DcMotorEx.Direction.REVERSE);
        DR4Bright.setDirection(DcMotorEx.Direction.REVERSE);

        DR4Bleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DR4Bright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        DR4Bleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DR4Bright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        DR4Bleft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        DR4Bright.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void init() {
        setPidPos("low");
//        DR4Bleft.setPower(-0.3);
//        DR4Bright.setPower(-0.3);

        telemetry.addData("DR4B", "Initialized");
        telemetry.update();
    }

    public void setPidPos(String location) {
        switch (location) {
            case "low": pid.setSetPoint(COUNTS_PER_DEGREE*0); break;
            case "high": pid.setSetPoint(COUNTS_PER_DEGREE*65); break;
            default: pid.setSetPoint(0);
        }
    }

    public double[] PID() {
        return new double[] {
                pid.calculate(DR4Bleft.getCurrentPosition()),
                pid.calculate(DR4Bright.getCurrentPosition())
        };
    }

    public void moveDR4B(double lPow, double rPow) {

        if (lPow < 0 || rPow < 0) {
            lPow = -0.3;
            rPow = -0.3;
        }

        if ((lPow > -0.2 && lPow < 0.2) || (rPow > -0.2 && rPow < 0.2)) {
            lPow = 0;
            rPow = 0;
        }

        DR4Bleft.setPower(lPow);
        DR4Bright.setPower(rPow);
    }

    public DcMotorEx[] motors() {
        return new DcMotorEx[] {
                this.DR4Bleft,
                this.DR4Bright
        };
    }

    @Override
    public void update() {
        DR4Bleft.setVelocity(PID()[0]);
        DR4Bright.setVelocity(PID()[1]);
    }


    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {

        lPow = -gp2.right_stick_y;
        rPow = -gp2.right_stick_y;

        if (-gp2.right_stick_y < 0) {
            lPow = 0;
            rPow = 0;
        }

        DR4Bleft.setPower(lPow);
        DR4Bright.setPower(rPow);
        telemetry.addData("rightDR4B", DR4Bright.getPower());
        telemetry.addData("leftDR4B", DR4Bleft.getPower());

    }
}
