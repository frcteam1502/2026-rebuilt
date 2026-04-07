package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonCameraCfg{	
	public static final String LEFT_APRILTAG_CAM = "leftAprilTagCam";
	public static final String RIGHT_APRILTAG_CAM = "rightAprilTagCam";
	public static final String RIGHT_HAMMER_APRILTAG_CAM = "rightHammerAprilTagCam";
	public static final String LEFT_HAMMER_APRILTAG_CAM = "leftHammerAprilTagCam";

	//Left AprilTag Cam Pose Config wrt robot center
	public static final double LEFT_APRILTAG_CAM_XPOS_METERS = -0.276;//"Forward" from center, in meters
	public static final double LEFT_APRILTAG_CAM_YPOS_METERS = 0.254;//"Left" from center, in meters
	public static final double LEFT_APRILTAG_CAM_ZPOS_METERS = 0.207;//"Up" from center, in meters
	
	public static final double LEFT_APRILTAG_CAM_ROLL_DEG	= 0;
	public static final double LEFT_APRILTAG_CAM_PITCH_DEG	= -15;
	public static final double LEFT_APRILTAG_CAM_YAW_DEG	= 155;
	
	public static final Transform3d LEFT_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(LEFT_APRILTAG_CAM_XPOS_METERS,
								      LEFT_APRILTAG_CAM_YPOS_METERS, 
									  LEFT_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(LEFT_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(LEFT_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(LEFT_APRILTAG_CAM_YAW_DEG)));

	//Left Hammer AprilTag Cam Pose Config wrt robot center
	public static final double LEFT_HAMMER_APRILTAG_CAM_XPOS_METERS = 0.314579;//"Forward" from center, in meters
	public static final double LEFT_HAMMER_APRILTAG_CAM_YPOS_METERS = 0.0417576;//"Left" from center, in meters
	public static final double LEFT_HAMMER_APRILTAG_CAM_ZPOS_METERS = 0.1813306;//"Up" from center, in meters
	
	public static final double LEFT_HAMMER_APRILTAG_CAM_ROLL_DEG	= 9.847;//Clockwise
	public static final double LEFT_HAMMER_APRILTAG_CAM_PITCH_DEG	= -110;
	public static final double LEFT_HAMMER_APRILTAG_CAM_YAW_DEG	= 118.024;
	
	public static final Transform3d LEFT_HAMMER_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(LEFT_HAMMER_APRILTAG_CAM_XPOS_METERS,
								      LEFT_HAMMER_APRILTAG_CAM_YPOS_METERS, 
									  LEFT_HAMMER_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(LEFT_HAMMER_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(LEFT_HAMMER_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(LEFT_HAMMER_APRILTAG_CAM_YAW_DEG)));

	
	//Right AprilTag Cam Pose Config wrt robot center
	public static final double RIGHT_APRILTAG_CAM_XPOS_METERS = -0.276;//"Forward" from center, in meters
	public static final double RIGHT_APRILTAG_CAM_YPOS_METERS = -0.254;//"Left" from center, in meters
	public static final double RIGHT_APRILTAG_CAM_ZPOS_METERS = 0.207;//"Up" from center, in meters
	
	public static final double RIGHT_APRILTAG_CAM_ROLL_DEG	= 0;
	public static final double RIGHT_APRILTAG_CAM_PITCH_DEG	= -15;
	public static final double RIGHT_APRILTAG_CAM_YAW_DEG	= -155;

	public static final Transform3d RIGHT_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(RIGHT_APRILTAG_CAM_XPOS_METERS,
								      RIGHT_APRILTAG_CAM_YPOS_METERS, 
									  RIGHT_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(RIGHT_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(RIGHT_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(RIGHT_APRILTAG_CAM_YAW_DEG)));

	//Right AprilTag Cam Pose Config wrt robot center
	public static final double RIGHT_HAMMER_APRILTAG_CAM_XPOS_METERS = -0.314579;//"Forward" from center, in meters
	public static final double RIGHT_HAMMER_APRILTAG_CAM_YPOS_METERS = -0.0417576;//"Left" from center, in meters
	public static final double RIGHT_HAMMER_APRILTAG_CAM_ZPOS_METERS = 0.1813306;//"Up" from center, in meters
	
	public static final double RIGHT_HAMMER_APRILTAG_CAM_ROLL_DEG	= -9.847;//CounterClockwise
	public static final double RIGHT_HAMMER_APRILTAG_CAM_PITCH_DEG	= -110;
	public static final double RIGHT_HAMMER_APRILTAG_CAM_YAW_DEG	= -118.024;

	public static final Transform3d RIGHT_HAMMER_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(RIGHT_HAMMER_APRILTAG_CAM_XPOS_METERS,
								      RIGHT_HAMMER_APRILTAG_CAM_YPOS_METERS, 
									  RIGHT_HAMMER_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(RIGHT_HAMMER_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(RIGHT_HAMMER_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(RIGHT_HAMMER_APRILTAG_CAM_YAW_DEG)));


	//Minimum abiguity to trust the pose (i.e. anything greater than this number discard)
	//public static final double MINIMUM_TARGET_AMBIGUITY = 0.25; 

	// The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    // public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    // public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);
	
	public static final AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltAndymark;
	//public static final AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltWelded;
	public static final AprilTagFieldLayout FIELD_TAG_LAYOUT = AprilTagFieldLayout.loadField(FIELD_VERSION);
	public static final double DISTANCE_THRESHOLD_M = 20;

  public static Transform3d robotToCamera0 = new Transform3d(-0.276, 0.254, 0.207, new Rotation3d(0.0, Math.toRadians(15), Math.toRadians(155)));
  public static Transform3d robotToCamera1 = new Transform3d(-0.276, -0.254, 0.207, new Rotation3d(0.0, Math.toRadians(15), Math.toRadians(-155)));

  public static Transform3d robotToCamera2 = new Transform3d(-0.250, 0.254, 0.207, new Rotation3d(0.0, Math.PI/12.0, Math.PI * 3.0 /4.0));
  public static Transform3d robotToCamera3 = new Transform3d(-0.250, -0.254, 0.207, new Rotation3d(0.0, Math.PI/12.0, -Math.PI* 3.0 /4.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.25;
  public static double maxZError = 1.12396;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0, // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

}