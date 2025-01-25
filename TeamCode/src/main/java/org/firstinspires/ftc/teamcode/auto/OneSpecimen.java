package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@Autonomous (name = "1+0", group = "Auto")
public class OneSpecimen extends OpMode {

    private Telemetry telemetryA;

    private Timer pathTimer, specTimer;

    private Follower f;

    private Slides slides;
    private IO io;

    private int autoState;

    private Path p1, p2;

    private final Pose startPose = new Pose(12, 58, 0);

    public static double pivInit = 600, pivDown = 1550, pivUp = 0, pivSpec = 400;
    public static double extIn = 0, extMid = 600, extOut = 2500;

    public static double pickDelay = 1.5, scoreDelay = 2;

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
        io.init();
        io.clawClose();

    }

    @Override
    public void init_loop() {
        slides.updatePiv();
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

        slides.updatePiv();
        io.update();

        telemetry.addData("pivPos ", slides.pivMotor().getCurrentPosition());
        telemetry.addData("extPos ", slides.spools()[0].getCurrentPosition());
        f.telemetryDebug(telemetryA);

    }

    public void setState(int state) { autoState = state; }

    public double pathTimer() { return pathTimer.getElapsedTimeSeconds(); }
    public double specTimer() { return specTimer.getElapsedTimeSeconds(); }

    public void pick( double delay) {
        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivUp);
            io.specimenInit();

            bool = true;
        }

        while (bool) {

            f.update();
            slides.updatePiv();
            io.update();

            if (specTimer() > 0 && specTimer() < delay) {

                if (slides.spools()[0].getCurrentPosition() > 400) slides.setSlidePower(-0.2);
                else slides.setSlidePower(0.5);

            }

            if (specTimer() > delay && specTimer() < delay + 1) {

                if (slides.spools()[0].getCurrentPosition() < 75) {
                    slides.setSlidePower(0);
                }
                else slides.setSlidePower(-0.5);
                io.clawClose();

            }

            if (specTimer() > delay + 0.5) {

                bool = false;
            }
        }
    }

    public void score(double delay) {

        if (!bool) {
            specTimer.resetTimer();
            slides.setPivTarget(pivUp);
            io.spec4auto();

            bool = true;
        }

        while (bool) {

            f.update();
            slides.updatePiv();
            io.update();


            if (specTimer() > delay && specTimer() < delay + 1.5) {

                if (slides.spools()[0].getCurrentPosition() > 1200) slides.setSlidePower(-0.2);
                else slides.setSlidePower(1);

                slides.setPivTarget(pivUp);
            }

            if (specTimer() > delay + 1.5) {
                if (slides.spools()[0].getCurrentPosition() < 10) {
                    slides.setSlidePower(0);
                }
                else slides.setSlidePower(-1);
                io.clawOpen();
                bool = false;
            }
        }
        io.specimenInit();
        slides.setSlidePower(-0.2);
    }

    public void build_paths() {
        p1 = new Path(
                new BezierCurve(
                        new Point(12.000, 58.000, Point.CARTESIAN),
                        new Point(20.000, 58.000, Point.CARTESIAN),
                        new Point(30.000, 68.000, Point.CARTESIAN),
                        new Point(36.000, 64.000, Point.CARTESIAN)
                )
        );
        p1.setConstantHeadingInterpolation(Math.toRadians(0));


        p2 = new Path(
                new BezierLine(
                        new Point(36.000, 64.000, Point.CARTESIAN),
                        new Point(18.000, 35.000, Point.CARTESIAN)
                )
        );
        p2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

    }

    public void autoUpdate() {
//        f.update();
        switch (autoState) {
            case 1:
                f.followPath(p1, true);
                setState(2);
                break;

            case 2:
                score(0.7);
                if (p1.isAtParametricEnd() || pathTimer() > 5) {
                    f.followPath(p2, true);
                    setState(3);
                }
                break;
            case 3:
                slides.setPivTarget(pivUp);
                io.straight();
                break;

        }
    }
}
