package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.DR4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "PedroTest", group = "Auto")
public class PedroTest extends OpMode {
    private Telemetry telemetryA;

    private Timer pathTimer;

    private Follower follower;

    private Intake intake;
    private Outtake outtake;

    private int autoState;

    private Path first, second, third;

    private Pose startPose = new Pose(0, 105, -180);

    private PIDController controller;

    private final double p = 0.01, i = 0, d = 0.0000003;

    private double dr4bTarget;

    private DR4B dr4b;

    public void setState(int state) {
        autoState = state;
    }

    public void autonomousPathUpdate() {
        switch (autoState) {
            case 1:
                follower.followPath(first);
                outtake.close();
                outtake.outSamp();
                intake.gbUp();
                intake.pivMid();
                setState(2);
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3.5) {
                    dr4bTarget = 650;
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.5 && pathTimer.getElapsedTimeSeconds() < 4) {
                    outtake.open();
                }
                if (pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 5) {
                    outtake.in();
                }
                if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 6) {
                    dr4bTarget = 300;
                }
                if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 6) {
                    dr4bTarget = 0;
                    setState(3);
                }
                break;
            case 3:
                if (first.isAtParametricEnd() || second.isAtParametricStart()) {
                    follower.followPath(second);
                    setState(4);
                }
                break;
            case 4:
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 5.75) {
                    dr4bTarget = 300;
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 5.75 && pathTimer.getElapsedTimeSeconds() < 6.25) {
                    dr4bTarget = 0;
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 6.25 && pathTimer.getElapsedTimeSeconds() < 6.75) {
                    intake.gbDown();
                    intake.pivDown();
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 6.75 && pathTimer.getElapsedTimeSeconds() < 7.25) {
                    intake.close();
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 7.25 && pathTimer.getElapsedTimeSeconds() < 8) {
                    intake.gbUp();
                    intake.pivUp();
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 8 && pathTimer.getElapsedTimeSeconds() < 8.5) {
                    intake.open();
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 8.5 && pathTimer.getElapsedTimeSeconds() < 8.75) {
                    outtake.close();
                    intake.gbDown();
                    intake.pivMid();
                }
                if (second.isAtParametricEnd() && pathTimer.getElapsedTimeSeconds() > 8.75 && pathTimer.getElapsedTimeSeconds() < 9) {
                    dr4bTarget =  dr4b.motors()[0].getCurrentPosition() + 650;
                    outtake.outSamp();
                    setState(5);
                }
                break;
            case 5:
                if(second.isAtParametricEnd() || third.isAtParametricStart()) {
                    follower.followPath(third);
                    setState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 11 && pathTimer.getElapsedTimeSeconds() < 11.25) {
                    outtake.open();
                }
                if (pathTimer.getElapsedTimeSeconds() > 11.25 && pathTimer.getElapsedTimeSeconds() < 11.75) {
                    intake.gbUp();
                    outtake.in();
                    setState(-1);
                }
                if (pathTimer.getElapsedTimeSeconds() < 11.75 && pathTimer.getElapsedTimeSeconds() < 12) {
                    dr4bTarget = 0;
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        follower = new Follower(hardwareMap);

        first = new Path(
                new BezierCurve(
                        new Point(9.992, 85.812, Point.CARTESIAN),
                        new Point(39.967, 107.363, Point.CARTESIAN),
                        new Point(39.591, 113.633, Point.CARTESIAN)
                )
        );
        first.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-85));
        first.setReversed(true);

        second = new Path(
                new BezierCurve(
                        new Point(39.591, 113.633, Point.CARTESIAN),
                        new Point(39.395, 112.849, Point.CARTESIAN),
                        new Point(40.962, 114.416, Point.CARTESIAN)
                )
        );
        second.setLinearHeadingInterpolation(Math.toRadians(-85), Math.toRadians(0));
        second.setReversed(false);

        third = new Path(
                new BezierCurve(
                        new Point(40.962, 114.416, Point.CARTESIAN),
                        new Point(42.726, 113.829, Point.CARTESIAN),
                        new Point(42.530, 116.376, Point.CARTESIAN)
                )
        );
        third.setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(0));
        third.setReversed(true);

        controller = new PIDController(p, i, d);

        intake = new Intake(hardwareMap, telemetryA);
        outtake = new Outtake(hardwareMap, telemetryA);
        dr4b = new DR4B(hardwareMap, telemetryA);

        setState(1);

        follower.setStartingPose(startPose);
        outtake.close();
    }

    @Override
    public void start() {
        intake.gbDown();
        intake.pivMid();
        outtake.in();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        follower.telemetryDebug(telemetryA);

        controller.setPID(p, i, d);
        int Pos = dr4b.motors()[0].getCurrentPosition();
        double Pid = controller.calculate(Pos, dr4bTarget);

        dr4b.motors()[0].setPower(Pid);
        dr4b.motors()[1].setPower(Pid);

        telemetryA.addData("Pos ", Pos);
        telemetryA.addData("Target ", dr4bTarget);
        telemetryA.update();
    }
}
