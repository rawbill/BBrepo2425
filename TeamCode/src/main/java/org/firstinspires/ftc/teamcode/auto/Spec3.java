package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing_old.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing_old.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@Autonomous (name = "3+0", group = "Auto")
public class Spec3 extends OpMode {

    private Telemetry telemetryA;

    private Timer pathTimer, specTimer;

    private Follower f;

    private Slides slides;
    private IO io;

    private int autoState;

    private Path p1, p2, p3, p4, p5, p6, p7, p8, p9;

    private final Pose startPose = new Pose(8, 56, 0);

    public static double pivInit = 600, pivDown = 1650, pivUp = 0, pivBack = -50;
    public static double extIn = 0, extMid = 600, extOut = 2500;

    public double extSpecI = 225, extSpecO = 1500;

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

        slides.setPivTarget(pivBack);

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

    public void pick(Path p, double d) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivBack);
            slides.setExtTarget(extSpecI);
            io.specimenInit();
            io.clawOpen();

            bool = true;
        }

        if (specTimer() > d-0.5) {
            io.clawClose();
        }

        if (specTimer() > d) {
            bool = false;
            setState(autoState+1);
        }
    }


    public void score(Path p, double d) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivBack);
            slides.setExtTarget(extIn);
            io.spec4auto();
            io.clawClose();

            bool = true;
        }

        if (p.isAtParametricEnd()) {
            slides.setExtTarget(extSpecO);

        }

        if (specTimer() > d) {
            slides.setExtTarget(extSpecI);
            io.clawOpen();
            bool = false;
            setState(autoState+1);
        }
    }

    public void build_paths() {
        p1 = new Path(
                new BezierCurve(
                        new Point(8.000, 56.000, Point.CARTESIAN),
                        new Point(18.000, 70.000, Point.CARTESIAN),
                        new Point(42.000, 68.000, Point.CARTESIAN)
                )
        );
        p1.setConstantHeadingInterpolation(Math.toRadians(0));


        p2 = new Path(
                new BezierCurve(
                        new Point(42.000, 68.000, Point.CARTESIAN),
                        new Point(4.000, 36.000, Point.CARTESIAN),
                        new Point(80.000, 34.000, Point.CARTESIAN),
                        new Point(64.000, 26.000, Point.CARTESIAN)
                )
        );
        p2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90));

        p3 = new Path(
                new BezierLine(
                        new Point(64.000, 26.000, Point.CARTESIAN),
                        new Point(18.000, 26.000, Point.CARTESIAN)
                )
        );
        p3.setConstantHeadingInterpolation(Math.toRadians(90));

        p4 = new Path(
                new BezierLine(
                        new Point(18.000, 26.000, Point.CARTESIAN),
                        new Point(25.000, 24.000, Point.CARTESIAN)
                )
        );
        p4.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0));

        p5 = new Path(
                new BezierLine(
                        new Point(25.000, 24.000, Point.CARTESIAN),
                        new Point(8.500, 24.000, Point.CARTESIAN)
                )
        );
        p5.setConstantHeadingInterpolation(Math.toRadians(0));


        p6 = new Path(
                new BezierCurve(
                        new Point(8.500, 24.000, Point.CARTESIAN),
                        new Point(18.000, 70.000, Point.CARTESIAN),
                        new Point(41.000, 70.000, Point.CARTESIAN)
                )
        );
        p6.setConstantHeadingInterpolation(Math.toRadians(0));

        p7 = new Path(
                new BezierLine(
                        new Point(41.000, 70.000, Point.CARTESIAN),
                        new Point(8.500, 24.000, Point.CARTESIAN)
                )
        );
        p7.setConstantHeadingInterpolation(Math.toRadians(0));

        p8 = new Path(
                new BezierCurve(
                        new Point(8.500, 24.000, Point.CARTESIAN),
                        new Point(18.000, 70.000, Point.CARTESIAN),
                        new Point(40.500, 73.000, Point.CARTESIAN)
                )
        );
        p8.setConstantHeadingInterpolation(Math.toRadians(0));

        p9 = new Path(
                new BezierLine(
                        new Point(40.500, 73.000, Point.CARTESIAN),
                        new Point(10.000, 20.000, Point.CARTESIAN)
                )
        );
        p9.setConstantHeadingInterpolation(Math.toRadians(0));

    }

    public void autoUpdate() {
//        f.update();
        switch (autoState) {
            case 1:
                f.followPath(p1, true);
                setState(2);
                break;
            case 2:
                score(p1, 3);
                break;
            case 3:
                if (p1.isAtParametricEnd()) {
                    f.followPath(p2, true);
                    setState(4);
                }
                break;
            case 4:
                if (p2.isAtParametricEnd()) {
                    f.followPath(p3, true);
                    setState(5);
                }
                break;
            case 5:
                if (p3.isAtParametricEnd()) {
                    f.followPath(p4, true);
                    setState(6);
                }
                pathTimer.resetTimer();
                break;
            case 6:
                if (pathTimer() > 2) setState(7);
                break;
            case 7:
                if (p4.isAtParametricEnd()) {
                    f.followPath(p5, true);
                    setState(8);
                }
            case 8:
                pick(p5, 3);
                break;
            case 9:
                if (p5.isAtParametricEnd()) {
                    f.followPath(p6, true);
                    setState(10);
                }
                break;
            case 10:
                score(p6, 3);
                break;
            case 11:
                if (p6.isAtParametricEnd()) {
                    f.followPath(p7, true);
                    setState(12);
                }
                break;
            case 12:
                pick(p7, 3);
                break;
            case 13:
                if (p7.isAtParametricEnd()) {
                    f.followPath(p8, true);
                    setState(14);
                }
                break;
            case 14:
                score(p8, 3);
                break;
            case 15:
                if (p8.isAtParametricEnd()) {
                    f.followPath(p9, true);
                    setState(16);
                }
                break;
            case 16:
                slides.setPivTarget(pivUp);
                slides.setExtTarget(extIn);
                io.straight();
                break;


        }
    }
}
