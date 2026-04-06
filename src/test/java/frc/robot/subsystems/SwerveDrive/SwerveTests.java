
package frc.robot.subsystems.SwerveDrive;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 *
 * @author jonat
 */
public class SwerveTests {
    @BeforeEach // this method will run before each test
    @SuppressWarnings("unused")
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }
    
    AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltAndymark;
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(FIELD_VERSION);
    
    
    @Test
    public void AutoAlignTest(){
        //frc.robot.commands.TurnToAngle cmd = new frc.robot.commands.TurnToAngle();
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        double l = fieldLayout.getFieldLength();
        double w = fieldLayout.getFieldWidth();

        Pose2d robotPose2d = new Pose2d();
        //robotPose2d = new Pose2d(new Translation2d(l,w),Rotation2d.kZero);
        Pose2d pose1 = fieldLayout.getTagPose(1).get().toPose2d();
        Translation2d tag1 = pose1.getTranslation();
        Translation2d targetPoint = tag1;
        Translation2d currentPosition = new Translation2d(robotPose2d.getX(), robotPose2d.getY());

        double x1 = currentPosition.getX();
        double y1 = currentPosition.getY();
        double x2 = targetPoint.getX();
        double y2 = targetPoint.getY();

        double delta_y = y2 - y1;
        double delta_x = x2 - x1;

        var angleRadians = Math.atan2(delta_y, delta_x);
        double degrees = angleRadians * 180 / Math.PI;

        var center = new Translation2d(l/2.0, w/2.0);
        var corner = new Pose2d(0, 0, Rotation2d.kZero);
        
        robotPose2d = updateRobotPose(robotPose2d,center);
  
        for (int i = 0; i < w; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX(), robotPose2d.getY()+1), robotPose2d.getRotation());
            robotPose2d = updateRobotPose(robotPose2d,center);
        }
        for (int i = 0; i < l; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX()+1, robotPose2d.getY()), robotPose2d.getRotation());
            robotPose2d = updateRobotPose(robotPose2d,center);
        }
        for (int i = 0; i < w; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX(), robotPose2d.getY()-1), robotPose2d.getRotation());
            robotPose2d = updateRobotPose(robotPose2d,center);
        }
        for (int i = 0; i < l; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX()-1, robotPose2d.getY()), robotPose2d.getRotation());
            robotPose2d = updateRobotPose(robotPose2d,center);
        }

        robotPose2d = new Pose2d();
        for (int i = 0; i < w; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX(), robotPose2d.getY()+1), robotPose2d.getRotation());
            updateRobotPose(robotPose2d,center);
        }
        for (int i = 0; i < l; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX()+1, robotPose2d.getY()), robotPose2d.getRotation());
            updateRobotPose(robotPose2d,center);
        }
        for (int i = 0; i < w; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX(), robotPose2d.getY()-1), robotPose2d.getRotation());
            updateRobotPose(robotPose2d,center);
        }
        for (int i = 0; i < l; i++) {
            robotPose2d = new Pose2d(new Translation2d(robotPose2d.getX()-1, robotPose2d.getY()), robotPose2d.getRotation());
            updateRobotPose(robotPose2d,center);
        }
    }

    /** Simulate slew to target */
    Pose2d updateRobotPose(Pose2d robotPose2d, Translation2d targetTranslation2d) {
        var delta = getRouteToTarget(robotPose2d, targetTranslation2d).getRotation(); 
        return new Pose2d(robotPose2d.getTranslation(),robotPose2d.getRotation().plus(delta));
    }
    /** distance + delta -- delta = getRoutToTarget().getRotation(), distance is .getTranslation.getNorm() */ 
    Pose2d getRouteToTarget(Pose2d robotPose2d, Translation2d targetTranslation2d) {
        var relativePose2d = (new Pose2d(targetTranslation2d, Rotation2d.kZero)).relativeTo(robotPose2d);
        Rotation2d delta = relativePose2d.getTranslation().getAngle();
        if (Math.abs(delta.getDegrees()) > 90.0) {
            delta = delta.rotateBy(Rotation2d.kPi);
        }

        var result = new Pose2d(relativePose2d.getTranslation(),delta);

        System.out.println(String.format("%-2.0f %2.0f   %+6.1f  %+6.1f    %+6.1f %+5.1f",
          robotPose2d.getX(),
          robotPose2d.getY(),
          robotPose2d.getRotation().getDegrees(),
          delta.getDegrees(),
          targetTranslation2d.minus(robotPose2d.getTranslation()).getAngle().getDegrees(),
          relativePose2d.getTranslation().getNorm())); 


        return result;
    }

    Rotation2d GetDelta(Pose2d robotPose2d, Pose2d aimPose2d)
    {
        return aimPose2d.getRotation().minus(robotPose2d.getRotation());
    }
}