package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing_old.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing_old.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@Autonomous (name = "1+0", group = "Auto")
public class Spec1 extends OpMode {

    private Telemetry telemetryA;

    private Timer pathTimer, specTimer;

    private Follower f;

    private Slides slides;
    private IO io;

    private int autoState;

    private Path p1, p2, p3;

    private final Pose startPose = new Pose(8, 56, 0);

    public static double pivInit = 600, pivDown = 1650, pivUp = 0, pivSpec = 400;
    public static double extIn = 0, extMid = 600, extOut = 2500;

    public double extSpec = 1500;

    public boolean bool = false;



    @Override
    public void init() {
        pathTimer = new Timer();
        specTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        f = new Follower(hardwareMap);

        slides = new Slides(hardwareMap, telemetryA);
        io = new IO(hardwareMap, telemetryA);

        build_paths();

        f.setStartingPose(startPose);

        slides.setPivTarget(pivInit);
        slides.setExtTarget(extIn);

        io.init();
        io.clawClose();

    }

    @Override
    public void init_loop() {
        slides.update();
        io.update();
        telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
        telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void start() {

        slides.setPivTarget(pivUp);

        setState(1);

        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        f.update();
        autoUpdate();

        slides.update();
        io.update();

        telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
        telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
        f.telemetryDebug(telemetryA);


    }

    public void setState(int state) { autoState = state; }

    public double pathTimer() { return pathTimer.getElapsedTimeSeconds(); }
    public double specTimer() { return specTimer.getElapsedTimeSeconds(); }


    public void score(Path p) {
        if (!bool) {
            slides.setPivTarget(pivUp);
            io.spec4auto();
            specTimer.resetTimer();

            bool = true;
        }

        if (p.isAtParametricEnd() || (specTimer() > 1.5 && specTimer() < 2.5)) {
            slides.setExtTarget(extSpec);

        }

        if (specTimer() > 2.5) {
            slides.setExtTarget(extIn);
            io.clawOpen();
            bool = false;
            setState(autoState+1);
        }
    }

    public void build_paths() {
        p1 = new Path(
                new BezierLine(
                        new Point(8.000, 56.000, Point.CARTESIAN),
                        new Point(8.000, 67.000, Point.CARTESIAN)
                )
        );
        p1.setConstantHeadingInterpolation(Math.toRadians(0));


        p2 = new Path(
                new BezierLine(
                        new Point(8.000, 67.000, Point.CARTESIAN),
                        new Point(35.000, 67.000, Point.CARTESIAN)
                )
        );
        p2.setConstantHeadingInterpolation(Math.toRadians(0));

        p3 = new Path(
                new BezierLine(
                        new Point(35.000, 67.000, Point.CARTESIAN),
                        new Point(15.000, 45.000, Point.CARTESIAN)
                )
        );
        p3.setConstantHeadingInterpolation(Math.toRadians(0));

    }

    public void autoUpdate() {
//        f.update();
        switch (autoState) {
            case 1:
                f.followPath(p1, true);
                setState(2);
                break;
            case 2:
                if (p1.isAtParametricEnd()) {
                    f.followPath(p2, true);
                    setState(3);
                }
                break;
            case 3:
                score(p2);
                break;
            case 4:
//                if (p2.isAtParametricEnd()) {
                    f.followPath(p3, true);
                    pathTimer.resetTimer();
                    setState(5);
//                }
                break;
            case 5:
                io.straight();
                if (pathTimer() > 2.5)
                    f.holdPoint(new BezierPoint(new Point(f.getPose().getX(), f.getPose().getY())), Math.toRadians(0));
                break;

        }
    }
}
