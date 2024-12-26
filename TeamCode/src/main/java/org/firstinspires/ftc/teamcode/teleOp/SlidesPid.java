package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name="Slidespid", group="TeleOp")
public class SlidesPid extends OpMode {

    private PIDController pivController;
    private PIDController extController;

    public static double pivP = 0, pivI = 0, pivD = 0;
    public static double extP = 0, extI = 0, extD = 0;

    public static int pivTarget = 0;
    public static int extTarget = 0;

    private Slides slides;

    @Override
    public void init() {
        pivController = new PIDController(pivP, pivTarget, pivD);
        extController = new PIDController(extP, extTarget, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides = new Slides(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        pivController.setPID(pivP, pivI, pivD);
        int pivPos = slides.pivMotor().getCurrentPosition();
        double pivPid = extController.calculate(pivPos, pivTarget);

        extController.setPID(extP, extI, extD);
        int extPos = slides.pivMotor().getCurrentPosition();
        double extPid = extController.calculate(extPos, extTarget);

        slides.pivMotor().setPower(pivPid);

        slides.spools()[0].setPower(extPid);
        slides.spools()[1].setPower(extPid);


        telemetry.addData("pivPos ", pivPos);
        telemetry.addData("pivTarget ", pivTarget);
        telemetry.addData("extPos ", extPos);
        telemetry.addData("extTarget ", extTarget);
        telemetry.update();

    }
}
