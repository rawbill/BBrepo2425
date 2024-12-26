package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DR4B;
import org.firstinspires.ftc.teamcode.subsystems.utilities.TelePIDConstants;

@Config
@TeleOp(name="DR4Bpid", group="TeleOp")
public class DR4Bpid extends OpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0000003;

    public static int target = 0;

    public final double TICKS_PER_DEGREE = 2786.2/360;
    // encoder resolution of 60 rpm yellow jacket divided by 360
    //       4 bar movement range is 85-150 degrees, or 0-65 degrees

    private DR4B dr4b;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dr4b = new DR4B(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int Pos = dr4b.motors()[0].getCurrentPosition();
        double Pid = controller.calculate(Pos, target);

        dr4b.motors()[0].setPower(Pid);
        dr4b.motors()[1].setPower(Pid);

        telemetry.addData("Pos ", Pos);
        telemetry.addData("Target ", target);
        telemetry.update();

    }
}
