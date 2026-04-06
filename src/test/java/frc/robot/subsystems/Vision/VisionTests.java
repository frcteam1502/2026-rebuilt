package frc.robot.subsystems.Vision;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.maxZError;

public class VisionTests {
    AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltAndymark;
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(FIELD_VERSION);
    
    
    @Test
    public void MaxZTest(){
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        var tags = fieldLayout.getTags();
        double maxz = 0.0;
        for (AprilTag tag : tags) {
            double z = tag.pose.getZ();
            if (maxz < z )
            {
                maxz = z;
                System.out.println("Max Z is: " + maxz);
            }
            Assertions.assertTrue(z < maxZError);
        }
    }
}
