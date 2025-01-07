package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides implements Subsystem {

    Telemetry telemetry;
    private DcMotorEx slidePiv;
    private DcMotorEx lSpool;
    private DcMotorEx rSpool;

    private PIDFController pivController;
    private PIDController extController;

    public static double Kcos = 0.0004;
    public static double pivP = 0.02, pivI = 0, pivD = 0.0002, pivF;
    public static double extP = 0.01, extI = 0, extD = 0.0001;

    public static double pivTarget = 0;
    public static double extTarget = 0;

    public Slides(HardwareMap map, Telemetry telemetry) {

        pivController = new PIDController(pivP, pivTarget, pivD);
        extController = new PIDController(extP, extTarget, extD);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidePiv = map.get(DcMotorEx.class, "slidepiv");
        lSpool = map.get(DcMotorEx.class, "lspool");
        rSpool = map.get(DcMotorEx.class,"rspool");

        slidePiv.setDirection(DcMotor.Direction.REVERSE);
        slidePiv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidePiv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePiv.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lSpool.setDirection(DcMotor.Direction.FORWARD);
        lSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rSpool.setDirection(DcMotor.Direction.REVERSE);
        rSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public DcMotorEx pivMotor() {
        return slidePiv;
    }

    public void setSlidePower(double power) {
        lSpool.setPower(power);
        rSpool.setPower(power);
    }
    public void setPivPower(double power) {
        slidePiv.setPower(power);
    }

    public DcMotorEx[] spools() {
        return new DcMotorEx[] {
                lSpool,
                rSpool
        };
    }

    @Override
    public void init() {

        telemetry.addData("Slides","Initialized");
        telemetry.update();
    }

    public void pivotSlides(double pow) {
        slidePiv.setPower(pow);
    }

   public void setPivTarget(double piv) {
        pivTarget = piv;
   }

   public void setExtTarget(double ext) {
        extTarget = ext;
   }

   public void updatePiv() {
       pivF = Math.cos(pivTarget) * Kcos;
       pivController.setPIDF(pivP, pivI, pivD, pivF);
       double pivPos = pivMotor().getCurrentPosition();
       double pivPid = extController.calculate(pivPos, pivTarget);

       pivMotor().setPower(pivPid);
   }

   public void updateExt() {
       extController.setPID(extP, extI, extD);
       double extPos = pivMotor().getCurrentPosition();
       double extPid = extController.calculate(extPos, extTarget);

       spools()[0].setPower(extPid);
       spools()[1].setPower(extPid);
   }

    @Override
    public void update() {
        pivF = Math.cos(pivTarget) * Kcos;
        pivController.setPIDF(pivP, pivI, pivD, pivF);
        double pivPos = pivMotor().getCurrentPosition();
        double pivPid = extController.calculate(pivPos, pivTarget);

        extController.setPID(extP, extI, extD);
        double extPos = pivMotor().getCurrentPosition();
        double extPid = extController.calculate(extPos, extTarget);

        pivMotor().setPower(pivPid);
        spools()[0].setPower(extPid);
        spools()[1].setPower(extPid);

    }

//    @Override
//    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
//        if (-gp2.left_stick_y < 0.1) pivTarget += -gp2.left_stick_y * 10;
//        else if (-gp2.left_stick_y > -0.1) pivTarget -= -gp2.left_stick_y * 10;
//
//        if (-gp2.right_stick_y < 0.1) extTarget += -gp2.right_stick_y * 10;
//        else if (-gp2.right_stick_y > -0.1) extTarget -= -gp2.right_stick_y * 10;
//
//    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        setSlidePower(-gp2.left_stick_y);
//        setPivPower(gp2.right_stick_y);
    }
}
