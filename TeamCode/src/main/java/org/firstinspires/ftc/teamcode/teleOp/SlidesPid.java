package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name="Slidespid", group="TeleOp")
public class SlidesPid extends LinearOpMode {

    public static double pivP = 0, pivI = 0, pivD = 0, pivF = 0;
    public static double extP = 0, extI = 0, extD = 0;

    public static int pivTarget = 0;
    public static int extTarget = 0;

    Slides slides;

    @Override
    public void runOpMode() throws InterruptedException {
        PIDFController pivController = new PIDFController(pivP, pivI, pivD, pivF);
        PIDController extController = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides = new Slides(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            int pivPos = slides.pivMotor().getCurrentPosition();
            double pivPid = pivController.calculate(pivPos, pivTarget);

            int extPos = slides.spools()[0].getCurrentPosition();
            double extPid = extController.calculate(extPos, extTarget);

            slides.setPivPower(pivPid);

            slides.setSlidePower(extPid);


            telemetry.addData("pivPos ", pivPos);
            telemetry.addData("pivTarget ", pivTarget);
            telemetry.addData("extPos ", extPos);
            telemetry.addData("extTarget ", extTarget);
            telemetry.update();
        }
    }
}
