package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AutoRobot;

@Config
@Autonomous(name = "0+4 RR", group = "Auto")
public class Sample4RR extends LinearOpMode {

    Telemetry telemetryA;

    public Timer pathTimer, specTimer;

    Pose2d initialPose = new Pose2d(-32, -62.5, Math.toRadians(90));
    MecanumDrive follower;

    AutoRobot autoRobot;

    Action score1, pick2, score2, pick3, score3, pick4, score4, ascend;

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
                                        score1,
                                        autoRobot.sampleScore(3.75)
                                ),
                                pick2,
                                autoRobot.samplePick(2.25, false),
                                new ParallelAction(
                                        score2,
                                        autoRobot.sampleScore(3.75)
                                ),
                                new ParallelAction(
                                        pick3,
                                        autoRobot.samplePick(3, false)
                                ),
                                new ParallelAction(
                                        score3,
                                        autoRobot.sampleScore(3.75)
                                ),
                                new ParallelAction(
                                        pick4,
                                        autoRobot.samplePick(3, true)
                                ),
                                new ParallelAction(
                                        score4,
                                        autoRobot.sampleScore(4.75)
                                ),
                                ascend,
                                autoRobot.rest()
                        ),
                        autoRobot.updateAction()
                )
        );

    }

    public void build_paths() {
        TrajectoryActionBuilder score1Path = follower.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45));

        TrajectoryActionBuilder pick2Path = score1Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47.75, -40.125), Math.toRadians(90));

        TrajectoryActionBuilder score2Path = pick2Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45));

        TrajectoryActionBuilder pick3Path = score2Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -41.25), Math.toRadians(90));

        TrajectoryActionBuilder score3Path = pick3Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45));

        TrajectoryActionBuilder pick4Path = score3Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53, -25.75), Math.toRadians(180));

        TrajectoryActionBuilder score4Path = pick4Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, -25), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(55));

        TrajectoryActionBuilder ascendPath = score4Path.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-36, -12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-16, -12), Math.toRadians(180));

        score1 = score1Path.build();
        pick2  = pick2Path.build();
        score2 = score2Path.build();
        pick3  = pick3Path.build();
        score3 = score3Path.build();
        pick4  = pick4Path.build();
        score4 = score4Path.build();
        ascend = ascendPath.build();

    }
}
