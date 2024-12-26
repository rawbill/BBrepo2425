package org.firstinspires.ftc.teamcode.zOldStuff.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmSubsystem {
    private DcMotor arm1, arm2;
    private static final int OUTTAKE_POS_TICKS = 160;
    private static final int INTAKE_POS_TICKS = 25;
    private static final double ARM_POWER = 0.2; // Adjust this value as needed

    public ArmSubsystem(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotor.class, "a1");
        arm2 = hardwareMap.get(DcMotor.class, "a2");

        arm1.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        // Perform any initialization tasks here, such as setting initial positions
        moveArmIntake(); // Move the arm to the intake position as the default position
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        boolean intakeButton = gamepad2.y; // Example: Change this according to your button mapping
        boolean outtakeButton = gamepad2.a; // Example: Change this according to your button mapping

        if (outtakeButton) {
            moveArmOuttake();
        } else if (intakeButton) {
            moveArmIntake();
        }
    }

    private void moveArmOuttake() {
        moveArmToPosition(OUTTAKE_POS_TICKS);
    }

    private void moveArmIntake() {
        moveArmToPosition(INTAKE_POS_TICKS);
    }

    private void moveArmToPosition(int positionTicks) {
        arm1.setTargetPosition(positionTicks);
        arm2.setTargetPosition(positionTicks);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm1.setPower(ARM_POWER);
        arm2.setPower(ARM_POWER);

        while (arm1.isBusy() && arm2.isBusy()) {
            // Wait for motors to reach target position
        }

        arm1.setPower(0.15);
        arm2.setPower(0.15);


    }
}
