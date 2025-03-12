package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;

public class Drivetrain implements Subsystem {

    public DcMotorEx lfMotor;
    public DcMotorEx lbMotor;
    public DcMotorEx rfMotor;
    public DcMotorEx rbMotor;

    private final IMU imu;

    Telemetry telemetry;
    
    double y, x, rx;
    double lfPower, lbPower, rfPower, rbPower;
    double denominator;

    public Drivetrain(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lfMotor = map.get(DcMotorEx.class, "lfm");
        lbMotor = map.get(DcMotorEx.class, "lbm");
        rfMotor = map.get(DcMotorEx.class, "rfm");
        rbMotor = map.get(DcMotorEx.class, "rbm");

        imu = map.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            )
        );

        lfMotor.setDirection(DcMotorEx.Direction.REVERSE);
        lbMotor.setDirection(DcMotorEx.Direction.REVERSE);

        lfMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

    public IMU imu() { return imu; }

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
        if (gp2.left_bumper) {
            if (gp2.left_stick_button || gp2.right_stick_button) {
                setDriveVectors(-gp2.left_stick_y * 0.3, gp2.left_stick_x * 0.4, gp2.right_stick_x * 0.3);
            } else {
                setDriveVectors(-gp2.left_stick_y, gp2.left_stick_x, gp2.right_stick_x);
            }
        } else {
            if (gp1.right_trigger > 0.1) {
                setDriveVectors(-gp1.left_stick_y * 0.3, gp1.left_stick_x * 0.4, gp1.right_stick_x * 0.3);
            } else {
                setDriveVectors(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x);
            }
        }
    }
    
}
