package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .0029;
        TwoWheelConstants.strafeTicksToInches = .0029;
        TwoWheelConstants.forwardY = 7.5;
        TwoWheelConstants.strafeX = -5.25;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "lfm";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rfm";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }
}




