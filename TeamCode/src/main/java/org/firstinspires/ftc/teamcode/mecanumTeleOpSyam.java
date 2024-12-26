package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class mecanumTeleOpSyam extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        double leftPower = -gamepad1.left_stick_y; // Inverting Y-axis to match expected control
        double rightPower = -gamepad1.right_stick_y;


        waitForStart();
        while(opModeIsActive()) {
            double denominator = Math.max(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x), 1.0);

            double frontLeftPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y) + gamepad1.right_stick_x)/denominator;
            double backLeftPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y) + gamepad1.right_stick_x)/denominator;
            double frontRightPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y) - gamepad1.right_stick_x)/denominator;
            double backRightPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y) - gamepad1.right_stick_x)/denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);





            frontLeftMotor.setPower(leftPower);
            frontRightMotor.setPower(rightPower);


            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();


        }
    }
}
