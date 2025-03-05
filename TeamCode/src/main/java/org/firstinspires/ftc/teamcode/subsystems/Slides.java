package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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

    public PIDController pivController;
    public PIDController extController;
    
    public static double pivPu = 0.00625, pivIu = 0, pivDu = 0.00075, pivPd = 0.0005, pivId = 0.0, pivDd = 0.0;
    public static double extP = 0.01, extI = 0, extD = 0.0001;

    public double pivTarget = 0;
    public double extTarget = 0;

    public Slides(HardwareMap map, Telemetry telemetry) {

        pivController = new PIDController(pivPu, pivIu, pivDu);
        extController = new PIDController(extP, extI, extD);

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

    public void setExtPower(double power) {
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

   public void setPivTarget(double piv) {
        pivTarget = piv;
   }

   public void setExtTarget(double ext) {
        extTarget = ext;
   }

   public void updatePiv() {
       int pivPos = pivMotor().getCurrentPosition();
       
       if ((pivTarget - pivPos) < 0) pivController.setPID(pivPu, pivIu, pivDu);
       else pivController.setPID(pivPd, pivId, pivDd);
       
       double pivPid = pivController.calculate(pivPos, pivTarget);
       pivPid = Math.max(-1, Math.min(1, pivPid));

       pivMotor().setPower(pivPid);
   }

   public void updateExt() {
       extController.setPID(extP, extI, extD);
       double extPos = spools()[0].getCurrentPosition();
       double extPid = extController.calculate(extPos, extTarget);

       setExtPower(extPid);
   }

    @Override
    public void update() {
        int pivPos = pivMotor().getCurrentPosition();
        
        if ((pivTarget - pivPos) < 0) pivController.setPID(pivPu, pivIu, pivDu);
        else pivController.setPID(pivPd, pivId, pivDd);
        
        double pivPid = pivController.calculate(pivPos, pivTarget);
        pivPid = Math.max(-1, Math.min(1, pivPid));
        
        extController.setPID(extP, extI, extD);
        int extPos = spools()[0].getCurrentPosition();
        double extPid = extController.calculate(extPos, extTarget);
        
        setPivPower(pivPid);
        
        setExtPower(extPid);
        
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

        setExtPower(-gp2.left_stick_y);
    }
}
