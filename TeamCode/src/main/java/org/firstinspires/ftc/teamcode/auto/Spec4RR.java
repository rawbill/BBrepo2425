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
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AutoRobot;

@Config
@Autonomous(name = "4+0 RR", group = "Auto")
public class Spec4RR extends LinearOpMode {

    Telemetry telemetryA;

    public Timer pathTimer, specTimer;

    Pose2d initialPose = new Pose2d(15, -62.5, Math.toRadians(90));
    MecanumDrive follower;


    AutoRobot autoRobot;

    Action place1, pushesAndPick2, place2, pick3, place3, pick4, place4, park;

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        specTimer = new Timer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        follower = new MecanumDrive(hardwareMap, initialPose);

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
                                            autoRobot.waitSeconds(9),
                                            autoRobot.specimenPick(2)
                                        )
                                ),
                                new ParallelAction(
                                        place2,
                                        autoRobot.specimenScore(3.25)
                                ),
                                new ParallelAction(
                                        pick3,
                                        autoRobot.specimenPick(2.75)
                                ),
                                new ParallelAction(
                                        place3,
                                        autoRobot.specimenScore(3.25)
                                ),
                                new ParallelAction(
                                        pick4,
                                        autoRobot.specimenPick(2.75)
                                ),
                                new ParallelAction(
                                        place4,
                                        autoRobot.specimenScore(3.25)
                                ),
                                new ParallelAction(
                                        park,
                                        autoRobot.specimenPick(1),
                                        autoRobot.end()
                                )
                        ),
                        autoRobot.updateAction()
                )
        );

    }

    public void build_paths() {
        TrajectoryActionBuilder place1Path = follower.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(3, -28.625), Math.toRadians(90));

        TrajectoryActionBuilder pushesAndPick2Path = place1Path.endTrajectory().fresh()
            .setTangent(Math.toRadians(-90))
            .splineToConstantHeading(new Vector2d(26, -36), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(47, -14, Math.toRadians(180)), Math.toRadians(0))
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(47, -50, Math.toRadians(180)), Math.toRadians(-90)) // push 2
            
            .setTangent(Math.toRadians(90))
//            .splineToLinearHeading(new Pose2d(43, -30, Math.toRadians(180)), Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(56, -14, Math.toRadians(180)), Math.toRadians(0))
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(180)), Math.toRadians(-90)) // push 3

//                .setTangent(Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(52, -47, Math.toRadians(90)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(46.5, -64, Math.toRadians(90)), Math.toRadians(-90)); // pick 2

        TrajectoryActionBuilder place2Path = pushesAndPick2Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -28.625), Math.toRadians(90));

        TrajectoryActionBuilder pick3Path = place2Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46.5, -64.5), Math.toRadians(90));

        TrajectoryActionBuilder place3Path = pick3Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-0.5, -28.625), Math.toRadians(90));

        TrajectoryActionBuilder pick4Path = place3Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46.5, -64.5), Math.toRadians(90));

        TrajectoryActionBuilder place4Path = pick4Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1, -28.625), Math.toRadians(90));

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
