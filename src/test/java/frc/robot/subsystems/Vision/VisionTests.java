package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.ArrayList;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Quaternion;

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

    @Test
    public void PoseTest1() {
        AprilTagFieldLayout blueLayout = AprilTagFieldLayout.loadField(FIELD_VERSION);
        AprilTagFieldLayout redLayout = AprilTagFieldLayout.loadField(FIELD_VERSION);
        redLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        var blueTags = blueLayout.getTags();
        var redTags = redLayout.getTags();
        var blue7 = blueLayout.getTagPose(7);
        var blue9 = blueLayout.getTagPose(9);
        var blue10 = blueLayout.getTagPose(10);
        var red7 = redLayout.getTagPose(7);
        var red9 = redLayout.getTagPose(9);
        var red10 = redLayout.getTagPose(10);

        //var redHub = 
 
    }
    
    @Test
    public void PoseDrift1() {
        Transform3d pose = new Transform3d(new Translation3d(2.05,0.315,0.37),new Rotation3d(new Quaternion(0.522,-0.124,0.022,-0.8)));
        Transform3d altpose = Transform3d.kZero;
        List<TargetCorner> minAreaRectCorners = new ArrayList<>(List.of(
            new TargetCorner(),
            new TargetCorner(),
            new TargetCorner(),
            new TargetCorner()
        ));
        List<TargetCorner> detectedCorners = new ArrayList<>();
        PhotonTrackedTarget tgt2 = new PhotonTrackedTarget(
            0.0, //yaw
            0.0, //pitch
            0.0, //area
            0.0, //skew
            2, //fiducial
            0, // classId
            1.0f, // objDetectConf
            pose,
            altpose,
            0.01, // ambiguity
            minAreaRectCorners,
            detectedCorners
        );
    }
}
