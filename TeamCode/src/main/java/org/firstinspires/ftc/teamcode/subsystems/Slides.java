package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides implements Subsystem {
    
    Telemetry telemetry;
    private DcMotorEx slidePiv, lSpool, rSpool;
    
    public PIDController pivController, extController;
    
    public static double pivPu = 0.00625, pivIu = 0, pivDu = 0.00075, pivPd = 0.0005, pivId = 0.0, pivDd = 0.0;
    public static double extP = 0.01, extI = 0, extD = 0.0001;
    
    public double pivTarget = 0, extTarget = 0;
    
    public static double pivInit = 600, pivDown = 1750, pivUp = 0;
    public static double extIn = 0, extMid = 400, extOut = 2500, extAscend = 1000;
    public static double intakeCap = 1000;
    
    public boolean lsbPressed = false;
    
    public Slides(HardwareMap map, Telemetry telemetry) {
        
        pivController = new PIDController(pivPu, pivIu, pivDu);
        extController = new PIDController(extP, extI, extD);
        
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        slidePiv = map.get(DcMotorEx.class, "slidepiv");
        lSpool = map.get(DcMotorEx.class, "lspool");
        rSpool = map.get(DcMotorEx.class, "rspool");
        
        slidePiv.setDirection(DcMotorEx.Direction.REVERSE);
        slidePiv.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slidePiv.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidePiv.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        lSpool.setDirection(DcMotorEx.Direction.FORWARD);
        lSpool.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lSpool.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lSpool.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        rSpool.setDirection(DcMotorEx.Direction.REVERSE);
        rSpool.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rSpool.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rSpool.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        pivInit = 600;
        pivDown = 1750;
        pivUp = 0;
        
    }
    
    public void resetExt() {
        
        lSpool.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lSpool.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        rSpool.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rSpool.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
        
        telemetry.addData("Slides", "Initialized");
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
        double extPos = spools()[1].getCurrentPosition();
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
        int extPos = spools()[1].getCurrentPosition();
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
        
        if (!gp2.left_bumper) {
            
            setExtPower(-gp2.left_stick_y);
            
            if (gp2.left_stick_button && !lsbPressed) {
                lsbPressed = true;
                resetExt();
            } else if (!gp2.a) {
                lsbPressed = false;
            }
            
        }
        
        pivInit += -gp2.right_stick_y * 15;
        pivDown += -gp2.right_stick_y * 15;
        pivUp += -gp2.right_stick_y * 15;
    }
}
