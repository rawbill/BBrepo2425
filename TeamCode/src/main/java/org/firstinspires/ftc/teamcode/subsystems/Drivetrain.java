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

    private DcMotorEx lfMotor;
    private DcMotorEx lbMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx rbMotor;

    private final IMU imu;

    private Timer timer;

    Telemetry telemetry;

    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: GOBILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_MM   = 96.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    private int state;
    private double distance;

    public Drivetrain(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new Timer();

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

    public void setState(int state, double distance) {
        this.state = state;
        this.distance = distance;
    }

    public boolean stateMachine() {

        final int STILL = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4;

        switch (state) {
            case STILL:
                lfMotor.setPower(0);
                lbMotor.setPower(0);
                rfMotor.setPower(0);
                rbMotor.setPower(0);
                lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                return true;
            case FORWARD:
                if (lfMotor.getCurrentPosition() < distance*COUNTS_PER_MM*25.4) {
                    if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (-5)) {
                        lfMotor.setPower(0.3);
                        lbMotor.setPower(0.3);
                        rfMotor.setPower(0.7);
                        rbMotor.setPower(0.7);
                    } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (5)) {
                        lfMotor.setPower(0.7);
                        lbMotor.setPower(0.7);
                        rfMotor.setPower(0.3);
                        rbMotor.setPower(0.3);
                    } else {
                        lfMotor.setPower(0.5);
                        lbMotor.setPower(0.5);
                        rfMotor.setPower(0.5);
                        rbMotor.setPower(0.5);
                    }
                    lfMotor.setPower(0.5);
                    lbMotor.setPower(0.5);
                    rfMotor.setPower(0.5);
                    rbMotor.setPower(0.5);
                    return false;
                } else {
                    setState(STILL, 0);
                    return true;
                }
            case BACKWARD:
                if (lfMotor.getCurrentPosition() < distance*COUNTS_PER_MM*25.4) {
                    if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (-0.5)) {
                        lfMotor.setPower(-0.7);
                        lbMotor.setPower(-0.7);
                        rfMotor.setPower(-0.3);
                        rbMotor.setPower(-0.3);
                    } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (0.5)) {
                        lfMotor.setPower(-0.3);
                        lbMotor.setPower(-0.3);
                        rfMotor.setPower(-0.7);
                        rbMotor.setPower(-0.7);
                    } else {
                        lfMotor.setPower(-0.5);
                        lbMotor.setPower(-0.5);
                        rfMotor.setPower(-0.5);
                        rbMotor.setPower(-0.5);
                    }
                    return false;
                } else {
                    setState(STILL, 0);
                    return true;
                }
            case LEFT:
                if (lfMotor.getCurrentPosition() < distance*COUNTS_PER_MM*25.4) {
                    if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (-0.5)) {
                        lfMotor.setPower(0.3);
                        lbMotor.setPower(-0.7);
                        rfMotor.setPower(-0.3);
                        rbMotor.setPower(0.7);
                    } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (0.5)) {
                        lfMotor.setPower(0.7);
                        lbMotor.setPower(-0.3);
                        rfMotor.setPower(-0.7);
                        rbMotor.setPower(0.3);
                    } else {
                        lfMotor.setPower(0.5);
                        lbMotor.setPower(-0.5);
                        rfMotor.setPower(-0.5);
                        rbMotor.setPower(0.5);
                    }
                    return false;
                } else {
                    setState(STILL, 0);
                    return true;
                }
            case RIGHT:
                if (lfMotor.getCurrentPosition() < distance*COUNTS_PER_MM*25.4) {
                    if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (-0.5)) {
                        lfMotor.setPower(-0.7);
                        lbMotor.setPower(0.3);
                        rfMotor.setPower(0.7);
                        rbMotor.setPower(-0.3);
                    } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (0.5)) {
                        lfMotor.setPower(-0.3);
                        lbMotor.setPower(0.7);
                        rfMotor.setPower(0.3);
                        rbMotor.setPower(-0.7);
                    } else {
                        lfMotor.setPower(-0.5);
                        lbMotor.setPower(0.5);
                        rfMotor.setPower(0.5);
                        rbMotor.setPower(-0.5);
                    }
                    return false;
                } else {
                    setState(STILL, 0);
                    return true;
                }
            default: return false;
        }
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
        stateMachine();
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {

        if (gp1.right_trigger > 0.1) {
            double y = -gp1.left_stick_y * 0.2; // Remember, Y stick value is reversed
            double x =  gp1.left_stick_x * 0.4;
            double rx = gp1.right_stick_x * 0.2;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lfPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

            lfMotor.setPower(lfPower);
            lbMotor.setPower(lbPower);
            rfMotor.setPower(rfPower);
            rbMotor.setPower(rbPower);
        } else {
            double y = -gp1.left_stick_y; // Remember, Y stick value is reversed
            double x =  gp1.left_stick_x;
            double rx = gp1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lfPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

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

    public boolean imuForwardInches(double inches) {
        if (lfMotor.getCurrentPosition() < inches*COUNTS_PER_MM*25.4) {
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (-0.5)) {
                lfMotor.setPower(0.3);
                lbMotor.setPower(0.3);
                rfMotor.setPower(0.7);
                rbMotor.setPower(0.7);
            } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (0.5)) {
                lfMotor.setPower(0.7);
                lbMotor.setPower(0.7);
                rfMotor.setPower(0.3);
                rbMotor.setPower(0.3);
            } else {
                lfMotor.setPower(0.5);
                lbMotor.setPower(0.5);
                rfMotor.setPower(0.5);
                rbMotor.setPower(0.5);
            }
            return false;
        } else {
            lfMotor.setPower(0);
            lbMotor.setPower(0);
            rfMotor.setPower(0);
            rbMotor.setPower(0);

            lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            return true;
        }
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
        sleep((long)(inches*25.4*1.25));
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
        sleep((long)(inches*25.4*1.25));
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
        sleep((long)(inches*25.4*1.25));
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
        sleep((long)(inches*25.4*1.25));
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

            telemetry.addData("yaw:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (directionDeg - margin)) {
                lfMotor.setPower(margin/-2);
                lbMotor.setPower(margin/-2);
                rfMotor.setPower(margin/2);
                rbMotor.setPower(margin/2);
            } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (directionDeg + margin)) {
                lfMotor.setPower(margin/2);
                lbMotor.setPower(margin/2);
                rfMotor.setPower(margin/-2);
                rbMotor.setPower(margin/-2);
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
