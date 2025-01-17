package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slides;


/* NOTES
 * where 0 is straight up, 775 is perfect for all the way down
 * 400 ticks is for init position
 * 2500 ticks for all the way up
 * 600 ticks worth of extension
 */

@Config
@TeleOp(name="Slidespid", group="TeleOp")
public class SlidesPid extends LinearOpMode {

    public static double Kcos = 0.001;
    public static double pivP = 0.00375, pivI = 0, pivD = 0.0001, pivF;
    public static double extP = 0.01, extI = 0, extD = 0.0001;

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
            pivF = Math.cos(pivTarget) * Kcos;
            pivController.setPIDF(pivP, pivI, pivD, pivF);
            int pivPos = slides.pivMotor().getCurrentPosition();
            double pivPid = pivController.calculate(pivPos, pivTarget);

            extController.setPID(extP, extI, extD);
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
