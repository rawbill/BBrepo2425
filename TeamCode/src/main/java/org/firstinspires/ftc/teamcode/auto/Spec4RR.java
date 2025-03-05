package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.DoubleLocalizerDrive;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AutoRobot;

@Config
@Autonomous(name = "4+0 RR", group = "Auto")
public class Spec4RR extends LinearOpMode {

    Telemetry telemetryA;

    public Timer pathTimer, specTimer;

    Pose2d initialPose = new Pose2d(15, -62.5, Math.toRadians(90));
    MecanumDrive ogFollower;
    DoubleLocalizerDrive follower;

    AutoRobot autoRobot;

    Action place1, pushesAndPick2, place2, pick3, place3, pick4, place4, park;

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        specTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        ogFollower = new MecanumDrive(hardwareMap, initialPose);
        follower = new DoubleLocalizerDrive(hardwareMap, initialPose);

        autoRobot = new AutoRobot(hardwareMap, telemetryA);

        build_paths();

        Actions.runBlocking(
                autoRobot.init()
        );

        while (!isStopRequested() && !opModeIsActive()) {
            autoRobot.update();
            telemetry.addData("pivPos ", autoRobot.slides.pivMotor().getCurrentPosition());
            telemetry.addData("extPos ", autoRobot.slides.spools()[0].getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        place1,
                                        autoRobot.specimenScore(3)
                                ),
                                autoRobot.rest(),
                                new ParallelAction(
                                        pushesAndPick2,
                                        new SequentialAction(
//                                            autoRobot.waitSeconds(0.5),
//                                            autoRobot.pivDown(AutoRobot.pivDown),
                                            autoRobot.waitSeconds(10),
                                            autoRobot.specimenPick(2)
                                        )
                                ),
                                new ParallelAction(
                                        place2,
                                        autoRobot.specimenScore(3)
                                ),
                                new ParallelAction(
                                        pick3,
                                        autoRobot.specimenPick(3)
                                ),
                                new ParallelAction(
                                        place3,
                                        autoRobot.specimenScore(3)
                                ),
                                new ParallelAction(
                                        pick4,
                                        autoRobot.specimenPick(3)
                                ),
                                new ParallelAction(
                                        place4,
                                        autoRobot.specimenScore(3)
                                ),
                                new ParallelAction(
                                        park,
                                        autoRobot.straight()
                                )
                        ),
                        autoRobot.updateAction()
                )
        );

    }

    public void build_paths() {
        TrajectoryActionBuilder place1Path = follower.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(3, -28.75), Math.toRadians(90));

        TrajectoryActionBuilder pushesAndPick2Path = place1Path.endTrajectory().fresh()
                //                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(43, -30, Math.toRadians(180)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56, -14, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(56, -54, Math.toRadians(180)), Math.toRadians(-90)) // push 3

//                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(54, -30, Math.toRadians(180)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(64.5, -14, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(64.5, -54, Math.toRadians(180)), Math.toRadians(-90)) // push 4

//                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(52, -47, Math.toRadians(90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(46.5, -62, Math.toRadians(90)), Math.toRadians(-90)); // pick 2

        TrajectoryActionBuilder place2Path = pushesAndPick2Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(1, -28.75), Math.toRadians(90));

        TrajectoryActionBuilder pick3Path = place2Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90));

        TrajectoryActionBuilder place3Path = pick3Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0.5, -28.75), Math.toRadians(90));

        TrajectoryActionBuilder pick4Path = place3Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46.5, -62), Math.toRadians(90));

        TrajectoryActionBuilder place4Path = pick4Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -28.75), Math.toRadians(90));

        TrajectoryActionBuilder parkPath = place4Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(90));

        place1         = place1Path.build();
        pushesAndPick2 = pushesAndPick2Path.build();
        place2         = place2Path.build();
        pick3          = pick3Path.build();
        place3         = place3Path.build();
        pick4          = pick4Path.build();
        place4         = place4Path.build();
        park           = parkPath.build();

    }
}
