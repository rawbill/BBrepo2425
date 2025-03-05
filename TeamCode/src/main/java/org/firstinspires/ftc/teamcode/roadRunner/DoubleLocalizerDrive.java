package org.firstinspires.ftc.teamcode.roadRunner;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadRunner.messages.PoseMessage;

public class DoubleLocalizerDrive extends MecanumDrive{
    private final TwoDeadWheelLocalizer twoDeadWheelLocalizer;
    private final OTOSLocalizer otosLocalizer;
    
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    
    public DoubleLocalizerDrive(HardwareMap map, Pose2d pose) {
        super(map, pose);
        this.twoDeadWheelLocalizer = new TwoDeadWheelLocalizer(map, super.lazyImu.get(), super.PARAMS.inPerTick, pose);
        this.otosLocalizer = new OTOSLocalizer(map, pose);
    }
    
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d twoWheelVel = twoDeadWheelLocalizer.update(); // X and Y
        PoseVelocity2d otosVel = otosLocalizer.update(); // Heading
        
        // Update estimated pose
        Pose2d pose = new Pose2d(
            twoDeadWheelLocalizer.getPose().position.x,
            twoDeadWheelLocalizer.getPose().position.y,
            otosLocalizer.getPose().heading.toDouble()
        );
        
        // Maintain pose history
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        
        estimatedPoseWriter.write(new PoseMessage(pose));
        
        return new PoseVelocity2d(
            twoWheelVel.linearVel,
            otosVel.angVel
        );
    }
}
