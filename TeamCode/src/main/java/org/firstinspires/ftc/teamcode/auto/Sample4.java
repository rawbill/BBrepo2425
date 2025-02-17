package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@Autonomous (name = "0+4", group = "Auto")
public class Sample4 extends OpMode {

    private Telemetry telemetryA;

    private Timer pathTimer, specTimer;

    private Follower f;

    private Slides slides;
    private IO io;

    private int autoState;

    private Path p1, p2, p3, p4, p5, p6, p7, p8;

    private final Pose startPose = new Pose(8, 104, 0);

    public static double pivInit = 600, pivDown = 1700, pivUp = 0, pivSpec = 400;
    public static double extIn = 0, extMid = 600, extOut = 2475;

    public double extSpec = 1500;

    public boolean bool = false;



    @Override
    public void init() {
        pathTimer = new Timer();
        specTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        Constants.setConstants(FConstants.class, LConstants.class);


        f = new Follower(hardwareMap);

        slides = new Slides(hardwareMap, telemetryA);
        io = new IO(hardwareMap, telemetryA);

        build_paths();

        f.setStartingPose(startPose);
        f.setMaxPower(1);

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

    public void pick(double d, boolean rotate) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setExtTarget(extIn);
            slides.setPivTarget(pivDown);
            io.gbPos = io.gbSetter(slides.spools()[0].getCurrentPosition(), 0.025);
            io.pivPos = io.pivSetter(slides.spools()[0].getCurrentPosition(), 0.025);

            if (rotate) io.rotPos = 0.8;
            else io.rotPos = 0.5;

            bool = true;
        }

        if (specTimer() > d - 0.5 && specTimer() < d) {
            io.clawClose();
        }

        if (specTimer() > d) {

            bool = false;

            setState(autoState+1);
        }
    }

    public void score(double d) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivUp);
            io.straight();

            bool = true;
        }

        if (bool && (specTimer() > 1 && specTimer() < d - 1)) {
            slides.setExtTarget(extOut);

        }

        if (specTimer() > d - 1.5 && specTimer() < d - 1) {
            io.outtakeInit();
            io.clawOpen();

        }

        if (specTimer() > d - 1) {
            io.intakeInit();
            slides.setExtTarget(extIn);
        }

        if (specTimer() > d) {

            bool = false;

            setState(autoState+1);
        }
    }

    public void build_paths() {
        p1 = new Path(
                new BezierLine(
                        new Point(8.000, 104.000, Point.CARTESIAN),
                        new Point(22.000, 127.000, Point.CARTESIAN)
                )
        );
        p1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45)); // score 1


        p2 = new Path(
                new BezierLine(
                        new Point(22.000, 127.000, Point.CARTESIAN),
                        new Point(31.000, 125.000, Point.CARTESIAN)
                )
        );
        p2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0)); // pick 1

        p3 = new Path(
                new BezierLine(
                        new Point(31.000, 125.000, Point.CARTESIAN),
                        new Point(24.000, 132.000, Point.CARTESIAN)
                )
        );
        p3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45)); // score 2

        p4 = new Path(
                new BezierLine(
                        new Point(24.000, 132.000, Point.CARTESIAN),
                        new Point(31.000, 132.000, Point.CARTESIAN)
                )
        );
        p4.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0)); // pick 2

        p5 = new Path(
                new BezierLine(
                        new Point(31.000, 132.000, Point.CARTESIAN),
                        new Point(24.000, 132.000, Point.CARTESIAN)
                )
        );
        p5.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45)); // score 3

        p6 = new Path(
                new BezierLine(
                        new Point(24.000, 132.000, Point.CARTESIAN),
                        new Point(41.000, 123.000, Point.CARTESIAN)
                )
        );
        p6.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90)); // pick 3

        p7 = new Path(
                new BezierCurve(
                        new Point(41.000, 123.000, Point.CARTESIAN),
                        new Point(39.000, 113.000, Point.CARTESIAN),
                        new Point(24.000, 132.000, Point.CARTESIAN)
                )
        );
        p7.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-45)); // score 4

        p8 = new Path(
                new BezierCurve(
                        new Point(24.000, 132.000, Point.CARTESIAN),
                        new Point(68.000, 130.000, Point.CARTESIAN),
                        new Point(60.000, 84.000, Point.CARTESIAN)
                )
        );
        p8.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90)); // ascend



    }

    public void autoUpdate() {
//        f.update();
        switch (autoState) {
            case 1:
                f.followPath(p1, true);
                setState(2);
                break;

            case 2:
                score(4);
                break;
            case 3:
                if (p1.isAtParametricEnd()) {
                    f.followPath(p2, true);
                    setState(4);
                }
                break;
            case 4:
                pick(3, false);
                break;
            case 5:
                if (p2.isAtParametricEnd()) {
                    f.followPath(p3, true);
                    setState(6);
                }
                break;
            case 6:
                score(4);
                break;
            case 7:
                if (p3.isAtParametricEnd()) {
                    f.followPath(p4, true);
                    setState(8);
                }
                break;
            case 8:
                pick(3, false);
                break;
            case 9:
                if (p4.isAtParametricEnd()) {
                    f.followPath(p5, true);
                    setState(10);
                }
                break;
            case 10:
                score(4);
                break;
            case 11:
                if (p5.isAtParametricEnd()) {
                    f.followPath(p6, true);
                    setState(12);
                }
                break;
            case 12:
                pick(3, true);
                break;
            case 13:
                if (p6.isAtParametricEnd()) {
                    f.followPath(p7, true);
                    setState(14);
                }
                break;
            case 14:
                score(4);
                break;
            case 15:
                if (p7.isAtParametricEnd()) {
                    f.followPath(p8, true);
                    setState(16);
                }
                break;
            case 16:
                io.outtakeInit();
                break;

        }
    }
}
