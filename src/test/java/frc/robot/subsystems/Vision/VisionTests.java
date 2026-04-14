package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static frc.robot.subsystems.Shooter.ShooterCfg.BLUE_HUB_TARGET_POSE;
import static frc.robot.subsystems.Shooter.ShooterCfg.RED_HUB_TARGET_POSE;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.RIGHT_APRILTAG_CAM_TRANSFORM;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.RIGHT_HAMMER_APRILTAG_CAM_TRANSFORM;
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
    public void TargetCheck() {
        AprilTagFieldLayout andymark = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        AprilTagFieldLayout welded = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        
        Pose3d redHubAndy = GetHubRed(andymark);
        Pose3d redHubWeld = GetHubRed(welded);
        Translation2d hubR = RED_HUB_TARGET_POSE;
        System.out.println("|RED  andymark x diff " + String.format("%+f", toInches(redHubAndy.getX()-hubR.getX())));
        System.out.println("|RED    welded x diff " + String.format("%+f", toInches(redHubWeld.getX()-hubR.getX())));
        System.out.println("|RED  andymark y diff " + String.format("%+f", toInches(redHubAndy.getY()-hubR.getY())));
        System.out.println("|RED    welded y diff " + String.format("%+f", toInches(redHubWeld.getY()-hubR.getY())));
        
        Translation2d hubB = BLUE_HUB_TARGET_POSE;
        Pose3d blueHubAndy = GetHubBlue(andymark);
        Pose3d blueHubWeld = GetHubBlue(welded);
        
        System.out.println("|BLUE andymark x diff " + String.format("%+f", toInches(blueHubAndy.getX()-hubB.getX())));
        System.out.println("|BLUE   welded x diff " + String.format("%+f", toInches(blueHubWeld.getX()-hubB.getX())));
        System.out.println("|BLUE andymark y diff " + String.format("%+f", toInches(blueHubAndy.getY()-hubB.getY())));
        System.out.println("|BLUE   welded y diff " + String.format("%+f", toInches(blueHubWeld.getY()-hubB.getY())));

    }
    double toInches(double meters) { return (meters * 100) / 2.54;}

    Pose3d GetHubRed(AprilTagFieldLayout layout) {
        var x = layout.getTagPose(5).get().getX();
        var y = layout.getTagPose(10).get().getY();
        var z = layout.getTagPose(10).get().getZ();
        return new Pose3d(x,y,z, Rotation3d.kZero);
    }
    Pose3d GetHubBlue(AprilTagFieldLayout layout) {
        var x = layout.getTagPose(18).get().getX();
        var y = layout.getTagPose(26).get().getY();
        var z = layout.getTagPose(26).get().getZ();
        return new Pose3d(x,y,z, Rotation3d.kZero);
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
        AprilTagFieldLayout blueLayout = AprilTagFieldLayout.loadField(FIELD_VERSION);
        var poseRotation = new Rotation3d(new Quaternion(0.522,-0.124,0.022,-0.8));
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
        
        var target = tgt2;
        var proll = Math.toDegrees(poseRotation.getX());
        var ppitch = Math.toDegrees(poseRotation.getY());
        var pyaw = Math.toDegrees(poseRotation.getZ());

        var robotToCamera = RIGHT_HAMMER_APRILTAG_CAM_TRANSFORM;
         

        /*robotToCamera = new Transform3d(
            new Translation3d(
                -Meters.convertFrom(10.898, Inches),
                -Meters.convertFrom(10.025, Inches),
                Meters.convertFrom(8.161, Inches)), 
            new Rotation3d(Math.toRadians(0), Math.toRadians(-15), Math.toRadians(-155)));*/

          var croll = Math.toDegrees(robotToCamera.getRotation().getX());
          var cpitch = Math.toDegrees(robotToCamera.getRotation().getY());
          var cyaw = Math.toDegrees(robotToCamera.getRotation().getZ());
        var tagPose = blueLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          var troll = Math.toDegrees(fieldToTarget.getRotation().getX());
          var tpitch = Math.toDegrees(fieldToTarget.getRotation().getY());
          var tyaw = Math.toDegrees(fieldToTarget.getRotation().getZ());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          var rroll = Math.toDegrees(cameraToTarget.getRotation().getX());
          var rpitch = Math.toDegrees(cameraToTarget.getRotation().getY());
          var ryaw = Math.toDegrees(cameraToTarget.getRotation().getZ());
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          var froll = Math.toDegrees(fieldToCamera.getRotation().getX());
          var fpitch = Math.toDegrees(fieldToCamera.getRotation().getY());
          var fyaw = Math.toDegrees(fieldToCamera.getRotation().getZ());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
          var roll = Math.toDegrees(robotPose.getRotation().getX());
          var pitch = Math.toDegrees(robotPose.getRotation().getY());
          var yaw = Math.toDegrees(robotPose.getRotation().getZ());
          var stop = true;
        }
        
    }
    
    @Test
    public void tgt7() {
        AprilTagFieldLayout blueLayout = AprilTagFieldLayout.loadField(FIELD_VERSION);
        Pose3d tgt7 = blueLayout.getTagPose(7).get();
        var tgt7b = tgt7.plus(new Transform3d(0.0,-0.02,0.0, Rotation3d.kZero));
        var rightHammerCam = new TestCamera("RightHammer", RIGHT_HAMMER_APRILTAG_CAM_TRANSFORM);
        var rightAprilCam = new TestCamera("RightApril ", RIGHT_APRILTAG_CAM_TRANSFORM);
        
        var photonTarget = new Transform3d(new Translation3d(3.691, -1.489, -0.477) , new Rotation3d(new Quaternion(-0.204, -0.142, -0.055, -0.967)));
        //Transform3d fieldToTarget = new Transform3d(tgt7.getTranslation(), tgt7.getRotation());
        Transform3d fieldToTarget = new Transform3d(tgt7b.getTranslation(), tgt7b.getRotation());
        Transform3d cameraToTarget = photonTarget;
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(rightHammerCam.GetRobotToCamera().inverse());
        Pose3d estimatedrobotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());


        Pose3d robotPose = new Pose3d(new Translation3d(16.5 - 0.33, 0.422, 0), new Rotation3d(0,0, -Math.PI/2));
        var ftr = new Transform3d(new Translation3d(16.5 - 0.33, 0.422, 0), new Rotation3d(0,0, -Math.PI/2));

        Pose3d robotToTarget = tgt7.relativeTo(robotPose);
        Pose2d rb = robotToTarget.toPose2d();
        System.out.println("robot       to Target: " + robotToTarget.toString());

        Pose3d rightHamerPose = rightHammerCam.GetCameraToField(robotPose);
        Pose3d rightAprilPose = rightAprilCam.GetCameraToField(robotPose);
        
        Pose3d rightHammerToTarget = rightHammerCam.GetCameraToTarget(robotPose, tgt7);
        Pose3d rightAprilToTarget = rightAprilCam.GetCameraToTarget(robotPose, tgt7);

        robotToTarget.getX();
        var done = true;
    }

    class TestCamera {
        String _name;
        Transform3d _robotToCamera;
        public TestCamera(String name, Transform3d robotToCamera) {
            _name = name;
            _robotToCamera = robotToCamera;
        }
        Transform3d GetRobotToCamera() { return _robotToCamera;}
        Pose3d GetCameraToField(Pose3d robotPose) { 
            var pose = robotPose.plus(_robotToCamera);
            System.out.println(_name + "  to Field: " +  pose.toString());
            return  pose;
        }
        //Transform3d GetCameraToTarget(Pose3d robotPose, Pose3d tgt) {
        Pose3d GetCameraToTarget(Pose3d robotPose, Pose3d tgt) {
            Pose3d cameraToRobot = robotPose.plus(_robotToCamera); 
            var pose = tgt.relativeTo(cameraToRobot);
            System.out.println(_name + " to Target: " +  pose.toString());
            return pose;
        }
    
    }
}
/*
 *  tgt7 11.94, .63, .89 (.87) .644 -- 2cm short
 *  from 0.33/.646, 0.422/0.466, 
 *            (16.17,  0.422) 
 * robot pose (16.268, 0.610) -1.527 : 3.704, -1.495, -0.479 (-0.198, -0.119, -0.060, -0.971) April * (-6.843, 0, 21.978) - pitch skew yaw
 * robot pose (16.276, 0.669) -1.528 : 3.691, -1.489, -0.477 (-0.204, -0.142, -0.055, -0.967) Hammer * (-6.840, 0, 21.973) - pitch skew yaw
 * robot pose (16.276, 0.669) -1.528 : 3.694, -1.491, -0.478 (-0.211, -0.131, -0.060, -0.967) Hammer * (-6.840, 0, 21.974) - pitch skew yaw
 * robot pose (16.276, 0.669) -1.528 : 3.694, -1.491, -0.478 (-0.211, -0.131, -0.060, -0.967) Hammer * (-6.840, 0, 21.974) - pitch skew yaw
 * robot pose (16.277, 0.664) -1.528 : 3.707, -1.496, -0.479 (-0.198, -0.118, -0.061, -0.971) Hammer * (-6.840, 0, 21.975) - pitch skew yaw
 * 
 * 
 * mutlitag LEFT
 * robot pose (16.171, 0.388) -1.569 : 4.300,  2.494, -0.287 (-0.213, -0.139, -0.041, -0.966) (-3.310, 0, -30.122) multi
 *                                  5:                                                         -3.310     -30.122
 *                                  8: 4.157,  2.172, -0.966 (-0.213, -0.139, -0.041, -0.966) (-2.887, 0, -27.560)
 *                                  9:                                                         -3.259     -23.347
 *                                 10:                                                         -4.172     -20.253
 *  5,8,9,10
 */