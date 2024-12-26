package org.firstinspires.ftc.teamcode.zOldStuff.Subsystems.Utilities;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDtutorial {
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    double Kg = 0;
    double lastTime = System.nanoTime();
    Telemetry t;
    private double lastError = 0;

    public PIDtutorial(double p, double i, double d, double f, double kg, Telemetry t) {
        Kp = p;
        Ki = i;
        Kd = d;
        Kf = f;
        Kg = kg;
        this.t = t;
    }

    public void calculatePID(double ticks, double currentTicks){

    }

    public double calculate(double reference, double state) {
        double currTime = System.nanoTime();
        double opt = System.nanoTime() + 9;
        double error = state - reference;
        double dt = currTime - lastTime;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        double gravity = Math.cos(getGroundAngle(state));
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (Math.signum(error) * Kf) + gravity * Kg;
        t.addData("Gravity", gravity * Kg);
        t.addData("Prop", error * Kp);
        t.addData("Der", derivative * Kd);
        t.addData("out", output);
        t.addData("useless", opt);
        t.addData("ground angle", getGroundAngle(state));
        lastError = error;
        lastTime = currTime;
        return output;
    }

    public double getGroundAngle(double ref){
        double conversion = 180.0 /(230+20);
        return conversion * (ref+20);
    }


}
