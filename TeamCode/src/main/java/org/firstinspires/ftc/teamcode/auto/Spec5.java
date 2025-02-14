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
@Autonomous (name = "5+0", group = "Auto")
public class Spec5 extends OpMode {

    private Telemetry telemetryA;

    private Timer pathTimer, specTimer;

    private Follower follower;

    private Slides slides;
    private IO io;

    private int autoState;

    private Path p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17;

    private final Pose startPose = new Pose(9, 56, Math.toRadians(0));

    public static double pivInit = 600, pivDown = 1650, pivUp = 0, pivBack = -50;
    public static double extIn = 0, extMid = 600, extOut = 2500;

    public double extSpecI = 225, extSpecO = 1500;

    public static double pickDelay = 1.5, scoreDelay = 2;

    public boolean bool = false;



    @Override
    public void init() {
        pathTimer = new Timer();
        specTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        slides = new Slides(hardwareMap, telemetryA);
        io = new IO(hardwareMap, telemetryA);

        build_paths();

        follower.setStartingPose(startPose);

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
        follower.update();
        autoUpdate();

        slides.update();
        io.update();

        telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
        telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
        follower.telemetryDebug(telemetryA);

    }

    public void setState(int state) { autoState = state; }

    public double pathTimer() { return pathTimer.getElapsedTimeSeconds(); }
    public double specTimer() { return specTimer.getElapsedTimeSeconds(); }

    public void pick(double delay) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivBack);
            slides.setExtTarget(extSpecI);
            io.specimenInit();
            io.clawOpen();

            bool = true;
        }

        if (specTimer() > delay - 0.5) {
            io.clawClose();
        }

        if (specTimer() > delay) {
            bool = false;
//            setState(autoState+1);
        }
    }

    public void score(double delay) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivBack);
            slides.setExtTarget(extIn);
            io.spec4auto();
            io.clawClose();

            bool = true;
        }

        if (specTimer() > delay-1 && specTimer() < delay) {
            slides.setExtTarget(extSpecO);

        }

        if (specTimer() > delay) {
            slides.setExtTarget(extSpecI);
            io.clawOpen();
            bool = false;
//            setState(autoState+1);
        }
    }

    public void build_paths() {
        p1 = new Path(
                new BezierLine(
                        new Point(9.000, 56.000, Point.CARTESIAN),
                        new Point(34.000, 64.000, Point.CARTESIAN)
                )
        );
        p1.setConstantHeadingInterpolation(Math.toRadians(0));


        p2 = new Path(
                new BezierCurve(
                        new Point(34.000, 64.000, Point.CARTESIAN),
                        new Point(15.000, 18.000, Point.CARTESIAN),
                        new Point(60.000, 45.000, Point.CARTESIAN),
                        new Point(65.000, 25.000, Point.CARTESIAN),
                        new Point(60.000, 23.000, Point.CARTESIAN)
                )
        );
        p2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90));

        p3 = new Path(
                new BezierLine(
                        new Point(60.000, 23.000, Point.CARTESIAN),
                        new Point(20.000, 23.000, Point.CARTESIAN)
                )
        );
        p3.setConstantHeadingInterpolation(Math.toRadians(90));


        p4 = new Path(
                new BezierCurve(
                        new Point(20.000, 23.000, Point.CARTESIAN),
                        new Point(75.000, 30.000, Point.CARTESIAN),
                        new Point(60.000, 12.000, Point.CARTESIAN)
                )
        );
        p4.setConstantHeadingInterpolation(Math.toRadians(90));


        p5 = new Path(
                new BezierLine(
                        new Point(60.000, 12.000, Point.CARTESIAN),
                        new Point(20.000, 12.000, Point.CARTESIAN)
                )
        );
        p5.setConstantHeadingInterpolation(Math.toRadians(90));

        p6 = new Path(
                new BezierCurve(
                        new Point(20.000, 12.000, Point.CARTESIAN),
                        new Point(75.000, 15.000, Point.CARTESIAN),
                        new Point(60.000, 7.000, Point.CARTESIAN)
                )
        );
        p6.setConstantHeadingInterpolation(Math.toRadians(90));

        p7 = new Path(
                new BezierLine(
                        new Point(60.000, 7.000, Point.CARTESIAN),
                        new Point(20.000, 7.000, Point.CARTESIAN)
                )
        );
        p7.setConstantHeadingInterpolation(Math.toRadians(90));

        p8 = new Path(
                new BezierLine(
                        new Point(20.000, 7.000, Point.CARTESIAN),
                        new Point(30.000, 24.000, Point.CARTESIAN)
                )
        );
        p8.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0));

        p9 = new Path(
                new BezierLine(
                        new Point(30.000, 24.000, Point.CARTESIAN),
                        new Point(13.000, 24.000, Point.CARTESIAN)
                )
        );
        p9.setConstantHeadingInterpolation(Math.toRadians(0));

        p10 = new Path(
                new BezierCurve(
                        new Point(13.000, 24.000, Point.CARTESIAN),
                        new Point(10.000, 75.000, Point.CARTESIAN),
                        new Point(40.000, 68.000, Point.CARTESIAN)
                )
        );
        p10.setConstantHeadingInterpolation(Math.toRadians(0));


        p11 = new Path(
                new BezierLine(
                        new Point(40.000, 68.000, Point.CARTESIAN),
                        new Point(13.000, 24.000, Point.CARTESIAN)
                )
        );
        p11.setConstantHeadingInterpolation(Math.toRadians(0));


        p12 = new Path(
                new BezierCurve(
                        new Point(13.000, 24.000, Point.CARTESIAN),
                        new Point(10.000, 75.000, Point.CARTESIAN),
                        new Point(40.000, 70.000, Point.CARTESIAN)
                )
        );
        p12.setConstantHeadingInterpolation(Math.toRadians(0));


        p13 = new Path(
                new BezierLine(
                        new Point(40.000, 70.000, Point.CARTESIAN),
                        new Point(13.000, 24.000, Point.CARTESIAN)
                )
        );
        p13.setConstantHeadingInterpolation(Math.toRadians(0));


        p14 = new Path(
                new BezierCurve(
                        new Point(13.000, 24.000, Point.CARTESIAN),
                        new Point(10.000, 75.000, Point.CARTESIAN),
                        new Point(40.000, 72.000, Point.CARTESIAN)
                )
        );
        p14.setConstantHeadingInterpolation(Math.toRadians(0));

        p15 = new Path(
                new BezierLine(
                        new Point(40.000, 72.000, Point.CARTESIAN),
                        new Point(13.000, 24.000, Point.CARTESIAN)
                )
        );
        p15.setConstantHeadingInterpolation(Math.toRadians(0));


        p16 = new Path(
                new BezierCurve(
                        new Point(13.000, 24.000, Point.CARTESIAN),
                        new Point(10.000, 75.000, Point.CARTESIAN),
                        new Point(40.000, 74.000, Point.CARTESIAN)
                )
        );
        p16.setConstantHeadingInterpolation(Math.toRadians(0));

        p17 = new Path(
                new BezierLine(
                        new Point(40.000, 74.000, Point.CARTESIAN),
                        new Point(14.000, 20.000, Point.CARTESIAN)
                )
        );
        p17.setConstantHeadingInterpolation(Math.toRadians(0));

    }

    public void autoUpdate() {
//        f.update();
        switch (autoState) {
            case 1:
                follower.followPath(p1, true);
                setState(2);
                break;

            case 2:
//                score(0.75);
                if (p1.isAtParametricEnd()) {
                    follower.followPath(p2, true);
                    setState(3);
                }
                break;
            case 3:
                if (p2.isAtParametricEnd()) {
                    follower.followPath(p3, true);
                    setState(4);
                }
                break;
            case 4:
                if (p3.isAtParametricEnd()) {
                    follower.followPath(p4, true);
                    setState(5);
                }
                break;
            case 5:
                if (p4.isAtParametricEnd()) {
                    follower.followPath(p5, true);
                    setState(6);
                }
                break;
            case 6:
                if (p5.isAtParametricEnd()) {
                    follower.followPath(p6, true);
                    setState(7);
                }
                break;
            case 7:
                if (p6.isAtParametricEnd()) {
                    follower.followPath(p7, true);
                    setState(8);
                }
                break;
            case 8:
                if (p7.isAtParametricEnd()) {
                    follower.followPath(p8, true);
                    setState(9);
                }
                break;
            case 9:
                if (p8.isAtParametricEnd()) {
                    follower.followPath(p9, true);
                    setState(10);
                }
                break;
            case 10:
//                pick(pickDelay);
                if (p9.isAtParametricEnd()) {
                    follower.followPath(p10, true);
                    setState(11);
                }
                break;
            case 11:
//                score(scoreDelay);
                if (p10.isAtParametricEnd()) {
                    follower.followPath(p11, true);
                    setState(12);
                }
                break;
            case 12:
//                pick(pickDelay + 0.25);
                if (p11.isAtParametricEnd()) {
                    follower.followPath(p12, true);
                    setState(13);
                }
                break;
            case 13:
//                score(scoreDelay);
                if (p12.isAtParametricEnd()) {
                    follower.followPath(p13, true);
                    setState(14);
                }
                break;
            case 14:
//                pick(pickDelay);
                if (p13.isAtParametricEnd()) {
                    follower.followPath(p14, true);
                    setState(15);
                }
                break;
            case 15:
//                score(scoreDelay);
                if (p14.isAtParametricEnd()) {
                    follower.followPath(p15, true);
                    setState(16);
                }
                break;
            case 16:
//                pick(pickDelay);
                if (p15.isAtParametricEnd()) {
                    follower.followPath(p16, true);
                    setState(17);
                }
                break;
            case 17:
//                score(scoreDelay);
                if (p16.isAtParametricEnd()) {
                    follower.followPath(p17, true);
                    setState(18);
                }
                break;
            case 18:
                slides.setPivTarget(pivUp);
                io.straight();
                break;
        }
    }
}
