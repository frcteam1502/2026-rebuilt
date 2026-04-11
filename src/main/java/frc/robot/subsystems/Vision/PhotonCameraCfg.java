package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class PhotonCameraCfg{	
	public static final String LEFT_APRILTAG_CAM = "leftAprilTagCam";
	public static final String RIGHT_APRILTAG_CAM = "rightAprilTagCam";
	public static final String RIGHT_HAMMER_APRILTAG_CAM = "rightHammerAprilTagCam";
	public static final String LEFT_HAMMER_APRILTAG_CAM = "leftHammerAprilTagCam";

	//Left AprilTag Cam Pose Config wrt robot center
	public static final double LEFT_APRILTAG_CAM_XPOS_METERS = -Meters.convertFrom(10.898, Inches);// -0.276;// 11.5"Forward" from center, in meters
	public static final double LEFT_APRILTAG_CAM_YPOS_METERS = Meters.convertFrom(10.025, Inches); //-0.254;// 13.5 - 2 3/8 "Left" from center, in meters
	public static final double LEFT_APRILTAG_CAM_ZPOS_METERS = Meters.convertFrom(8.161, Inches); //0.207;// 8.0 "Up" from center, in meters
	
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
	public static final double LEFT_HAMMER_APRILTAG_CAM_XPOS_METERS = -Meters.convertFrom(1.625, Inches); //"Forward" from center, in meters
	public static final double LEFT_HAMMER_APRILTAG_CAM_YPOS_METERS = Meters.convertFrom(12.460, Inches); //"Left" from center, in meters
	public static final double LEFT_HAMMER_APRILTAG_CAM_ZPOS_METERS = Meters.convertFrom(6.917, Inches);  //;//"Up" from center, in meters
	
	public static final double LEFT_HAMMER_APRILTAG_CAM_ROLL_DEG = 0;//Clockwise
	public static final double LEFT_HAMMER_APRILTAG_CAM_PITCH_DEG = -20;
	public static final double LEFT_HAMMER_APRILTAG_CAM_YAW_DEG	= 70;
	
	public static final Transform3d LEFT_HAMMER_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(LEFT_HAMMER_APRILTAG_CAM_XPOS_METERS,
								      LEFT_HAMMER_APRILTAG_CAM_YPOS_METERS, 
									  LEFT_HAMMER_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(LEFT_HAMMER_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(LEFT_HAMMER_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(LEFT_HAMMER_APRILTAG_CAM_YAW_DEG)));

	
	//Right AprilTag Cam Pose Config wrt robot center
	public static final double RIGHT_APRILTAG_CAM_XPOS_METERS = -Meters.convertFrom(10.898, Inches); //"Forward" from center, in meters
	public static final double RIGHT_APRILTAG_CAM_YPOS_METERS = -Meters.convertFrom(10.025, Inches); // "Left" from center, in meters
	public static final double RIGHT_APRILTAG_CAM_ZPOS_METERS = Meters.convertFrom(8.161, Inches);   // "Up" from center, in meters
	
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
	public static final double RIGHT_HAMMER_APRILTAG_CAM_XPOS_METERS = -Meters.convertFrom(1.625, Inches);  //-0.316479;//"Forward" from center, in meters
	public static final double RIGHT_HAMMER_APRILTAG_CAM_YPOS_METERS = -Meters.convertFrom(12.490, Inches); //"Left" from center, in meters
	public static final double RIGHT_HAMMER_APRILTAG_CAM_ZPOS_METERS = Meters.convertFrom(6.917, Inches);   //"Up" from center, in meters
	
	public static final double RIGHT_HAMMER_APRILTAG_CAM_ROLL_DEG	=   0; //CounterClockwise
	public static final double RIGHT_HAMMER_APRILTAG_CAM_PITCH_DEG	= -20;
	public static final double RIGHT_HAMMER_APRILTAG_CAM_YAW_DEG	= -70;

	public static final Transform3d RIGHT_HAMMER_APRILTAG_CAM_TRANSFORM = new Transform3d(
					new Translation3d(RIGHT_HAMMER_APRILTAG_CAM_XPOS_METERS,
								      RIGHT_HAMMER_APRILTAG_CAM_YPOS_METERS, 
									  RIGHT_HAMMER_APRILTAG_CAM_ZPOS_METERS), 
					new Rotation3d(Math.toRadians(RIGHT_HAMMER_APRILTAG_CAM_ROLL_DEG),
								   Math.toRadians(RIGHT_HAMMER_APRILTAG_CAM_PITCH_DEG), 
								   Math.toRadians(RIGHT_HAMMER_APRILTAG_CAM_YAW_DEG)));

	
	public static final AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltAndymark;
	//public static final AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltWelded;
	public static final AprilTagFieldLayout FIELD_TAG_LAYOUT = AprilTagFieldLayout.loadField(FIELD_VERSION);
	public static final double DISTANCE_THRESHOLD_M = 20;

	// Basic filtering thresholds
	public static double maxAmbiguity = 0.25;
	public static double maxZError = 1.12396;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static double linearStdDevBaseline = 0.2; // Meters
	public static double angularStdDevBaseline = 0.6; // Radians

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