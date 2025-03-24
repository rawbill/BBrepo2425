package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain implements Subsystem {
    
    public DcMotor lfMotor, lbMotor, rfMotor, rbMotor;
    
    Telemetry telemetry;
    
    double y = 0, x = 0, rx = 0, denominator = 0;
    public double lfPower = 0, lbPower = 0, rfPower = 0, rbPower = 0;
    
    public Drivetrain(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        lfMotor = map.get(DcMotor.class, "lfm");
        lbMotor = map.get(DcMotor.class, "lbm");
        rfMotor = map.get(DcMotor.class, "rfm");
        rbMotor = map.get(DcMotor.class, "rbm");
        
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    @Override
    public void init() {
        telemetry.addData("Drivetrain", "Initialized");
        telemetry.update();
    }
    
    public void setDriveVectors(double y, double x, double rx) {
        this.y = y;
        this.x = x;
        this.rx = rx;
    }
    
    @Override
    public void update() {
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lfPower = (y + x + rx) / denominator;
        lbPower = (y - x + rx) / denominator;
        rfPower = (y - x - rx) / denominator;
        rbPower = (y + x - rx) / denominator;
        
        lfMotor.setPower(lfPower);
        lbMotor.setPower(lbPower);
        rfMotor.setPower(rfPower);
        rbMotor.setPower(rbPower);
    }
    
    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (!gp2.left_bumper) {
            if (gp1.right_trigger > 0.1) {
                setDriveVectors(-gp1.left_stick_y * 0.3, gp1.left_stick_x * 0.4, gp1.right_stick_x * 0.3);
            } else {
                setDriveVectors(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x); // not supposed to be slow
            }
        } /* else {
            if (gp2.left_stick_button || gp2.right_stick_button) {
                setDriveVectors(-gp2.left_stick_y * 0.3, gp2.left_stick_x * 0.4, gp2.right_stick_x * 0.3);
            } else {
                setDriveVectors(-gp2.left_stick_y, gp2.left_stick_x, gp2.right_stick_x); // not supposed to be slow
            }
        } */
        update();
    }
    
}
