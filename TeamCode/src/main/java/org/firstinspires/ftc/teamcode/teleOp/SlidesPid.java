package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slides;


/* NOTES
 * where 0 is straight up, 1750 is perfect for all the way down
 * 600 ticks is for init position
 * 2500 ticks for all the way up
 * 600 ticks worth of extension
 */

@Config
@TeleOp(name="Slidespid", group="TeleOp")
public class SlidesPid extends LinearOpMode {
    
    public static double pivPu = 0.00625, pivIu = 0, pivDu = 0.00075, pivPd = 0.0005, pivId = 0.0, pivDd = 0.0;
    public static double extP = 0.01, extI = 0, extD = 0.0001;

    public static double pivTarget = 0;
    public static double extTarget = 0;

    Slides slides;

    @Override
    public void runOpMode() throws InterruptedException {

        PIDController pivController = new PIDController(pivPu, pivIu, pivDu);
        PIDController extController = new PIDController(extP, extI, extD);
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides = new Slides(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            
            int pivPos = slides.pivMotor().getCurrentPosition();
            
            if ((pivTarget - pivPos) < 0) pivController.setPID(pivPu, pivIu, pivDu);
            else pivController.setPID(pivPd, pivId, pivDd);
            
            double pivPid = pivController.calculate(pivPos, pivTarget);
            pivPid = Math.max(-1, Math.min(1, pivPid));

            extController.setPID(extP, extI, extD);
            int extPos = slides.spools()[0].getCurrentPosition();
            double extPid = extController.calculate(extPos, extTarget);
            extPid = Math.max(-1, Math.min(1, extPid));

            slides.setPivPower(pivPid);

            slides.setExtPower(extPid);
            
            telemetry.addData("pivPos", pivPos);
            telemetry.addData("pivTarget", pivTarget);
            telemetry.addData("pivPID Output", pivPid);
            telemetry.addData("pivError", pivTarget - pivPos);
            telemetry.addData("extPos ", extPos);
            telemetry.addData("extTarget ", extTarget);
            telemetry.update();
        }
    }
}
