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
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing_old.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing_old.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.IO;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@Autonomous (name = "AscentPark", group = "Auto")
public class AscentL1 extends OpMode {

    private Telemetry telemetryA;
    private Timer pathTimer;

    private Follower f;

    private Slides slides;
    private IO io;

    private int autoState;

    private Path p1;

    private final Pose startPose = new Pose(8, 104, 0);

    public static double pivInit = 600, pivDown = 1650, pivUp = 0, pivSpec = 400;
    public static double extIn = 0, extMid = 600, extOut = 2500;


    @Override
    public void init() {
        pathTimer = new Timer();

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

    public void build_paths() {
        p1 = new Path(
                new BezierCurve(
                        new Point(8.000, 104.000, Point.CARTESIAN),
                        new Point(72.000, 118.000, Point.CARTESIAN),
                        new Point(68.000, 100.000, Point.CARTESIAN)
                )
        );
        p1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90));

    }

    public void autoUpdate() {
        switch (autoState) {
            case 1:
                f.followPath(p1, true);
                setState(2);
                break;

            case 2:
                io.outtakeInit();
                break;

        }
    }
}
