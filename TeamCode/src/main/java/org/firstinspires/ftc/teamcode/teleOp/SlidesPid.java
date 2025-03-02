package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slides;


/* NOTES
 * where 0 is straight up, 1700 is perfect for all the way down
 * 600 ticks is for init position
 * 2500 ticks for all the way up
 * 600 ticks worth of extension
 */

@Config
@TeleOp(name="Slidespid", group="TeleOp")
public class SlidesPid extends LinearOpMode {
    
    public static double pivP = 0.01, pivI = 0, pivD = 0.0001, pivF, pivKcos = 0.15;
    public static double extP = 0.01, extI = 0, extD = 0.0001;
    
    public final double ticks_per_degree = 5281.1 / 360;

    public static double pivTarget = 0;
    public static double extTarget = 0;

    Slides slides;

    @Override
    public void runOpMode() throws InterruptedException {

        PIDController pivController = new PIDController(pivP, pivI, pivD);
        PIDController extController = new PIDController(extP, extI, extD);
        
//        ProfiledPIDController pivController = new ProfiledPIDController(pivP, pivI, pivD, new TrapezoidProfile.Constraints(pivMaxVel, pivMaxAccel));
//        ArmFeedforward pivFeedforward;
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides = new Slides(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            pivController.setPID(pivP, pivI, pivD);
//            pivController.setConstraints(new TrapezoidProfile.Constraints(pivMaxVel, pivMaxAccel));
            int pivPos = slides.pivMotor().getCurrentPosition();
            double pivPid = pivController.calculate(pivPos, pivTarget);
            pivF = Math.sin(Math.toRadians(pivTarget/ticks_per_degree)) * pivKcos;
//            pivFeedforward = new ArmFeedforward(pivKs, pivKcos, pivKv);
//            pivF = pivFeedforward.calculate(Math.toRadians(pivController.getSetpoint().position/ticks_per_degree), pivMaxVel, pivMaxAccel);
            double pivPower = pivPid + pivF;

            extController.setPID(extP, extI, extD);
            int extPos = slides.spools()[0].getCurrentPosition();
            double extPid = extController.calculate(extPos, extTarget);

            slides.setPivPower(pivPower);

            slides.setExtPower(extPid);

            telemetry.addData("pivPos ", pivPos);
            telemetry.addData("pivTarget ", pivTarget);
            telemetry.addData("pivPower" , pivPower);
            telemetry.addData("extPos ", extPos);
            telemetry.addData("extTarget ", extTarget);
            telemetry.update();
        }
    }
}
