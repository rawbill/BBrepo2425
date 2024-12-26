package org.firstinspires.ftc.teamcode.zOldStuff.Subsystems;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain implements Subsystem{

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    Telemetry telemetry;

    private static final double TICKS_PER_REV = 537.7; // Adjust this value based on your motor
    private static final double DRIVE_GEAR_REDUCTION = 1; // This is > 1.0 if geared up
    private static final double WHEEL_DIAMETER_INCHES = 3.77953; // For example, adjust for your wheel size
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    // a comment in java
    private BHI260IMU imu;
    private double lastYaw; // Last recorded Yaw angle
    private double globalAngle; // Global angle for orientation
    private double kP = 0.1;
    private double kD = 0.1;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        frontLeftMotor = hardwareMap.dcMotor.get("lfm");
        backLeftMotor = hardwareMap.dcMotor.get("lbm");
        frontRightMotor = hardwareMap.dcMotor.get("rfm");
        backRightMotor = hardwareMap.dcMotor.get("rbm");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x =  -gamepad1.right_stick_x;
        double rx = -gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // Method to move the robot forward/backward a certain number of feet
    public void move(char direction, double inches, double power) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH + 9.75);

        if(direction == 'F'){
            setTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);
        }
        else if(direction == 'B'){
            setTargetPosition(-moveCounts, -moveCounts, -moveCounts, -moveCounts);
        }
        setMotorPower(power);
        waitForMotors();
    }

    public void moveIMU(char direction, double inches, double power) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH + 9.75);
        double startAngle = getAngle(); // Get the current IMU angle
        double lastError = 0; // To store the last error for derivative calculation

        // Set target positions
        if(direction == 'F'){
            setTargetPosition(moveCounts, moveCounts, moveCounts, moveCounts);
        } else if(direction == 'B'){
            setTargetPosition(-moveCounts, -moveCounts, -moveCounts, -moveCounts);
        }

        // Start moving
        setMotorPower(power);

        // Loop until motors reach the target position
        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
            double currentAngle = getAngle();
            double error = startAngle - currentAngle; // Calculate the error from the start angle
            double derivative = error - lastError; // Calculate the derivative of the error

            // Adjust motor powers based on the error and derivative
            double correction = kP * error + kD * derivative;

            // Check if the error exceeds 10 degrees
            if (Math.abs(error) > 10) {
                // Report the error and stop the program
                telemetry.addData("Error", "Correction exceeds 10 degrees: " + error);

                // Stop all motors
                setMotorPower(0);

                // Exit the method, effectively stopping the program
                return;
            }

            // Apply the correction differently to left and right sides
            double leftPower = power + correction;
            double rightPower = power - correction;

            // Apply corrected power to motors
            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(rightPower);

            lastError = error; // Update the last error for the next loop iteration

            // Optionally, add telemetry data here
        }

        // Stop all motors
        setMotorPower(0);
    }


    // Method to strafe left/right R--> right, L --> left
    public void strafe(char direction, double feet, double power) {
        int moveCounts = (int) (feet * 12 * COUNTS_PER_INCH + 45);
        if(direction == 'L'){
            setTargetPosition(moveCounts, moveCounts, -moveCounts, -moveCounts);
        }
        else if(direction == 'R'){
            setTargetPosition(-moveCounts, -moveCounts, moveCounts, moveCounts);
        }
        setMotorPower(power);
        waitForMotors();
    }

    public void strafeIMU(char direction, double feet, double power) {
        int moveCounts = (int) (feet * 12 * COUNTS_PER_INCH + 45);
        double startAngle = getAngle(); // Get the current IMU angle

        // Set target positions for strafing
        if(direction == 'L'){
            setTargetPosition(moveCounts, -moveCounts, -moveCounts, moveCounts);
        } else if(direction == 'R'){
            setTargetPosition(-moveCounts, moveCounts, moveCounts, -moveCounts);
        }

        // Start strafing
        setMotorPower(power);

        // Loop until motors reach the target position
        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
            double currentAngle = getAngle();
            double error = startAngle - currentAngle; // Calculate the error from the start angle

            // Adjust motor powers based on the error
            double correction = kP * error;
            double leftPower = (direction == 'L') ? power + correction : power - correction;
            double rightPower = (direction == 'L') ? power - correction : power + correction;

            frontLeftMotor.setPower(leftPower);
            backLeftMotor.setPower(rightPower);
            frontRightMotor.setPower(rightPower);
            backRightMotor.setPower(leftPower);
        }

        // Stop all motors
        setMotorPower(0);
    }

    // Method to turn the robot
    public void turn(char direction, double degrees, double power) {
        // Turning calculations will depend on your robot's specific turning radius
        // For example, if a 360 degree turn requires moving 5 feet for each wheel
        // Then, a 1 degree turn requires moving (5 feet / 360) for each wheel
        double turnFeet = (5.0 / 360) * degrees;
        int turnCounts = (int) (turnFeet * 12 * COUNTS_PER_INCH);
        // Witch negative sign based on what robot does
        turnCounts = (direction == 'L') ? -turnCounts : turnCounts;
        setTargetPosition(-turnCounts, turnCounts, -turnCounts, turnCounts);
        setMotorPower(power); // Set desired power level
        waitForMotors();
    }

    // Method to turn the robot using the IMU
    public void turnIMU(char direction, double degrees, double power) {
        resetAngle(); // Reset the IMU's angle tracking

        // Determine turn direction
        if (direction == 'L') {
            degrees *= -1; // Invert the degrees for left turn
        }

        setMotorPower((direction == 'L') ? -power : power);

        // Rotate until turn is completed
        if (degrees < 0) { // Left turn
            while (getAngle() > degrees) {
                // Update telemetry or add any other necessary code here
            }
        } else { // Right turn
            while (getAngle() < degrees) {
                // Update telemetry or add any other necessary code here
            }
        }

        // Turn off motors
        setMotorPower(0);

        // Reset angle tracking on new heading
        resetAngle();
    }

    private void resetAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        lastYaw = orientation.getYaw(AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
        double deltaAngle = currentYaw - lastYaw;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastYaw = currentYaw;

        return globalAngle;
    }


    private void setTargetPosition(int fl, int bl, int fr, int br) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + fl);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + bl);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + fr);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + br);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorPower(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    private void waitForMotors() {
        // Wait for all motors to reach their target positions
        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
            // Optionally, you can add some telemetry data here to monitor the motor progress
            // For example:
            // telemetry.addData("Motors", "Moving to position");
            // telemetry.update();
        }

        // Stop all motors once target positions are reached
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // Optionally, reset motor modes if necessary
        // For example, setting them back to RUN_USING_ENCODER or RUN_WITHOUT_ENCODER
    }


    @Override
    public void addTelemetry(Telemetry t, boolean shouldPrint) {

    }
}
