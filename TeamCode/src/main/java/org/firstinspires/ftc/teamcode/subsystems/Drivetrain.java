package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain implements Subsystem {

    private DcMotorEx lfMotor;
    private DcMotorEx lbMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx rbMotor;

    private final IMU imu;

    Telemetry telemetry;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: GOBILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_MM   = 96.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    boolean rbPressed;

    public Drivetrain(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lfMotor = map.get(DcMotorEx.class, "lfm");
        lbMotor = map.get(DcMotorEx.class, "lbm");
        rfMotor = map.get(DcMotorEx.class, "rfm");
        rbMotor = map.get(DcMotorEx.class, "rbm");

        imu = map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

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

    public void power(double power) {
        lfMotor.setPower(power);
        lbMotor.setPower(power);
        rfMotor.setPower(power);
        rbMotor.setPower(power);
    }

    public IMU imu() {
        return imu;
    }

    @Override
    public void update() {

    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        double y = -gp1.left_stick_y; // Remember, Y stick value is reversed
        double x =  gp1.left_stick_x;
        double rx = gp1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lfPower = (y + x + rx) / denominator;
        double lbPower = (y - x + rx) / denominator;
        double rfPower = (y - x - rx) / denominator;
        double rbPower = (y + x - rx) / denominator;

        if (gp1.right_trigger > 0.1) {
            lfMotor.setPower(lfPower * 0.2);
            lbMotor.setPower(lbPower * 0.2);
            rfMotor.setPower(rfPower * 0.2);
            rbMotor.setPower(rbPower * 0.2);
        } else {
            lfMotor.setPower(lfPower);
            lbMotor.setPower(lbPower);
            rfMotor.setPower(rfPower);
            rbMotor.setPower(rbPower);
        }

    }

    public void startEncoder() {
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopEncoder() {
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void encoderDriveForwardInches(double inches) throws InterruptedException {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;

        lfMotor.setTargetPosition((int)TotalTicks);
        lbMotor.setTargetPosition((int)TotalTicks);
        rfMotor.setTargetPosition((int)TotalTicks);
        rbMotor.setTargetPosition((int)TotalTicks);
        lfMotor.setPower(0.5);
        lbMotor.setPower(0.5);
        rfMotor.setPower(0.5);
        rbMotor.setPower(0.5);
        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*25.4*2));
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveBackwardInches(double inches) throws InterruptedException {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;

        lfMotor.setTargetPosition(-((int)TotalTicks));
        lbMotor.setTargetPosition(-((int)TotalTicks));
        rfMotor.setTargetPosition(-((int)TotalTicks));
        rbMotor.setTargetPosition(-((int)TotalTicks));
        lfMotor.setPower(0.5);
        lbMotor.setPower(0.5);
        rfMotor.setPower(0.5);
        rbMotor.setPower(0.5);
        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*25.4*2));
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveLeftInches(double inches) throws InterruptedException {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;

        lfMotor.setTargetPosition(-(int)TotalTicks);
        lbMotor.setTargetPosition((int)TotalTicks);
        rfMotor.setTargetPosition((int)TotalTicks);
        rbMotor.setTargetPosition(-(int)TotalTicks);
        lfMotor.setPower(0.5);
        lbMotor.setPower(0.5);
        rfMotor.setPower(0.5);
        rbMotor.setPower(0.5);
        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*25.4*2));
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveRightInches(double inches) throws InterruptedException {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;

        lfMotor.setTargetPosition(((int)TotalTicks));
        lbMotor.setTargetPosition(-((int)TotalTicks));
        rfMotor.setTargetPosition(-((int)TotalTicks));
        rbMotor.setTargetPosition(((int)TotalTicks));
        lfMotor.setPower(0.5);
        lbMotor.setPower(0.5);
        rfMotor.setPower(0.5);
        rbMotor.setPower(0.5);
        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*25.4*2));
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void imuCorrection(double directionDeg, double margin) {

        lfMotor.setPower(0);
        lbMotor.setPower(0);
        rfMotor.setPower(0);
        rbMotor.setPower(0);

        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (
                !(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (directionDeg - margin) &&
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (directionDeg + margin))
        ) {

            telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (directionDeg - margin)) {
                lfMotor.setPower(margin/2);
                lbMotor.setPower(margin*-1/2);
                rfMotor.setPower(margin*-1/2);
                rbMotor.setPower(margin/2);
            } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (directionDeg + margin)) {
                lfMotor.setPower(margin*-1/2);
                lbMotor.setPower(margin/2);
                rfMotor.setPower(margin/2);
                rbMotor.setPower(margin*-1/2);
            } else {
                lfMotor.setPower(0);
                lbMotor.setPower(0);
                rfMotor.setPower(0);
                rbMotor.setPower(0);
            }
        }

        lfMotor.setPower(0);
        lbMotor.setPower(0);
        rfMotor.setPower(0);
        rbMotor.setPower(0);

        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
