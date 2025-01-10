package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.OTOS;

        FollowerConstants.leftFrontMotorName = "lfm";
        FollowerConstants.leftRearMotorName = "lbm";
        FollowerConstants.rightFrontMotorName = "rfm";
        FollowerConstants.rightRearMotorName = "rbm";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.548; // in kg

        FollowerConstants.xMovement = 61.7536; // 63.0639, 61.7867, 57.1772, 59.4895, 61.7536
        FollowerConstants.yMovement = 43.9856; // 42.2116, 41.2216, 47.5017, 46.46, 43.9856

        FollowerConstants.forwardZeroPowerAcceleration = -28.9039; // -29.8325, -27.7316, -28.9039, -28.7956, -30.1718
        FollowerConstants.lateralZeroPowerAcceleration = -78.2881; // -76.604, -78.2881, -80.2332, -77.0713, -80.2767

        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.001,0,0,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.001,0,0,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(0.0005,0,0,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(0.0005,0,0,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.1,0,0,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
