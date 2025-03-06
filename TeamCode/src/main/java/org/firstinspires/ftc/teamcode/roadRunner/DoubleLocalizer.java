package org.firstinspires.ftc.teamcode.roadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadRunner.messages.TwoDeadWheelInputsMessage;

@Config
public class DoubleLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = -2482.6444128525345; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -1404.9539991287384; // x position of the perpendicular encoder (in tick units)
        
        public double angularScalar = 1.0;
        public double linearScalar = 0.9882;
        
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.5, 6+13/16.0, Math.PI / 2);
    }
    
    public static Params PARAMS = new Params();
    
    public final SparkFunOTOS otos;
    private Pose2d otosPose;
    
    public final Encoder par, perp;
    public final IMU imu;
    
    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;
    
    private final double inPerTick;
    
    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;
    private Pose2d pose;
    
    public DoubleLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "lfm")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rfm")));
        
        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);
        
        this.imu = imu;
        
        this.inPerTick = inPerTick;
        
        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
        
        pose = initialPose;
        
        // TODO: make sure your config has an OTOS device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otosPose = initialPose;
        otos.setPosition(OTOSKt.toOTOSPose(otosPose));
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        
        otos.calibrateImu();
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.setOffset(PARAMS.offset);
    }
    
    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
        
        otosPose = pose;
        otos.setPosition(OTOSKt.toOTOSPose(otosPose));
    }
    
    @Override
    public Pose2d getPose() {
        return new Pose2d(
            pose.position,
            otosPose.heading
        );
    }
    
    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
            UnnormalizedAngleUnit.RADIANS,
            (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
            (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
            (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
            angularVelocityDegrees.acquisitionTime
        );
        
        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));
        
        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        
        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;
        
        if (!initialized) {
            initialized = true;
            
            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;
            
            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }
        
        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);
        
        Twist2dDual<Time> twist = new Twist2dDual<>(
            new Vector2dDual<>(
                new DualNum<Time>(new double[] {
                    parPosDelta - PARAMS.parYTicks * headingDelta,
                    parPosVel.velocity - PARAMS.parYTicks * headingVel,
                }).times(inPerTick),
                new DualNum<Time>(new double[] {
                    perpPosDelta - PARAMS.perpXTicks * headingDelta,
                    perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                }).times(inPerTick)
            ),
            new DualNum<>(new double[] {
                headingDelta,
                headingVel,
            })
        );
        
        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;
        
        pose = pose.plus(twist.value());
        
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);
        
        this.otosPose = OTOSKt.toRRPose(otosPose);
        Vector2d fieldVel = new Vector2d(otosVel.x, otosVel.y);
        Vector2d robotVel = fieldVel.times(otosVel.h);
        
        // use x and y of odo pods and heading of otos
        return new PoseVelocity2d(
            twist.velocity().value().linearVel,
            otosVel.h
        );
    }
}