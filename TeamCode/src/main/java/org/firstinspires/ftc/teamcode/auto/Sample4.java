package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing_old.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing_old.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;
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
            io.gbPos = io.gbSetter(slides.spools()[0].getCurrentPosition(), 0);
            io.pivPos = io.pivSetter(slides.spools()[0].getCurrentPosition(), 0);

            if (rotate) io.rotPos = 0.8;
            else io.rotPos = 0.5;

            bool = true;
        }

        if (specTimer() > d) {
            io.clawClose();
            slides.setPivTarget(pivUp);

            bool = false;

            setState(autoState+1);
        }
    }

    public void score(double d) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivUp);
            io.outtakeInit();

            bool = true;
        }

        if (bool) {
            slides.setExtTarget(extOut);

        }

        if (specTimer() > d-0.5) {
            io.clawOpen();
        }

        if (specTimer() > d) {
            slides.setExtTarget(extIn);

            bool = false;

            setState(autoState+1);
        }
    }

    public void build_paths() {
        p1 = new Path(
                new BezierLine(
                        new Point(8.000, 104.000, Point.CARTESIAN),
                        new Point(13.000, 131.000, Point.CARTESIAN)
                )
        );
        p1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));


        p2 = new Path(
                new BezierLine(
                        new Point(13.000, 131.000, Point.CARTESIAN),
                        new Point(30.000, 121.000, Point.CARTESIAN)
                )
        );
        p2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));

        p3 = new Path(
                new BezierLine(
                        new Point(30.000, 121.000, Point.CARTESIAN),
                        new Point(13.000, 131.000, Point.CARTESIAN)
                )
        );
        p3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        p4 = new Path(
                new BezierLine(
                        new Point(13.000, 131.000, Point.CARTESIAN),
                        new Point(30.000, 132.000, Point.CARTESIAN)
                )
        );
        p4.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));

        p5 = new Path(
                new BezierLine(
                        new Point(30.000, 132.000, Point.CARTESIAN),
                        new Point(13.000, 131.000, Point.CARTESIAN)
                )
        );
        p5.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        p6 = new Path(
                new BezierLine(
                        new Point(13.000, 131.000, Point.CARTESIAN),
                        new Point(45.500, 129.000, Point.CARTESIAN)
                )
        );
        p6.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));

        p7 = new Path(
                new BezierLine(
                        new Point(45.500, 129.000, Point.CARTESIAN),
                        new Point(13.000, 131.000, Point.CARTESIAN)
                )
        );
        p7.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-45));

        p8 = new Path(
                new BezierLine(
                        new Point(13.000, 131.000, Point.CARTESIAN),
                        new Point(68.000, 100.000, Point.CARTESIAN)
                )
        );
        p8.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));



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
                pick(1, false);
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
                pick(1, false);
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
                pick(1, true);
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
