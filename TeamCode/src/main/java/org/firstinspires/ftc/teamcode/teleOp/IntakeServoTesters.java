package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeServoTesters extends LinearOpMode {
    private Servo rightGb;
    private Servo leftGb;

    public void runOpMode() {
        rightGb = hardwareMap.get(Servo.class, "rightGb");
//        leftGb  = hardwareMap.get(Servo.class, "leftGb");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x) {
                rightGb.setPosition(1);
            }
            if (gamepad2.b) {
                rightGb.setPosition(0);
            }
        }
    }
}
