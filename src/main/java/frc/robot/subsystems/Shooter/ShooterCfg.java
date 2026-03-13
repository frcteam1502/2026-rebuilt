package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterCfg {
    //Shooter Motor Config
    public static final int LEAD_SHOOTER_MOTOR_ID = 1;
    public static final int FOLLOWER_SHOOTER_MOTOR_ID = 2;
    public static final int HOOD_MOTOR_ID = 6;
    public static final int FEED_MOTOR_ID = 7;
    public static final int INDEXER_MOTOR_ID = 12;
    public static final int TURRET_MOTOR_ID = 13;

    //CAN encoder IDs
    public static final int TURRET_ABS_ENCODER_ID = 13;
    public static final int HOOD_ABS_ENCODER_ID = 14;

    public static final SparkFlex LEAD_SHOOTER_MOTOR = new SparkFlex(LEAD_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex FOLLOWER_SHOOTER_MOTOR = new SparkFlex(FOLLOWER_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax HOOD_MOTOR = new SparkMax(HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax FEED_MOTOR = new SparkMax(FEED_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex INDEXER_MOTOR = new SparkFlex(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax TURRET_MOTOR = new SparkMax(TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final CANcoder TURRET_ABS_ENCODER = new CANcoder(TURRET_ABS_ENCODER_ID);
    public static final CANcoder HOOD_ABS_ENCODER = new CANcoder(HOOD_ABS_ENCODER_ID);

    //CanCoder Configuration 
    public static final double TURRET_ABS_ENCODER_OFFSET = -0.8624;
    public static final double HOOD_ABS_ENCODER_OFFSET = -0.9203;
    public static final SensorDirectionValue TURRET_CAN_CODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue HOOD_CAN_CODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    public static final double DISCONTINUITY_POINT = 1;

    //PIDF Gains
    public static final double SHOOTER_P_GAIN = 0.00075;
    public static final double SHOOTER_I_GAIN = 0;
    public static final double SHOOTER_D_GAIN = 0.004;
    public static final double SHOOTER_ALLOWED_ERROR = 100;

    public static final double TURRET_P_GAIN = 12;
    public static final double TURRET_I_GAIN = 0.001;   
    public static final double TURRET_D_GAIN = 0.005;

    public static final double HOOD_P_GAIN = 70;
    public static final double HOOD_I_GAIN = 0.1;
    public static final double HOOD_D_GAIN = 0;

    //Shooter
    public static final double SHOOTER_ENC_POS_CONFIG = 1;
    public static final double SHOOTER_ENC_VEL_CONFIG = 1;
    public static final double SHOOTER_KV = 0.002;
    public static final double SHOOTER_KA = 0.11812;
    public static final double SHOOTER_KS = 0.35577;

    public static final boolean SHOOTER_LEAD_INVERTED = false;
    public static final IdleMode SHOOTER_IDLE_MODE = IdleMode.kCoast;
    public static final int SHOOTER_CURRENT_LIMIT = 100;
    public static final boolean SHOOTER_FOLLOW_INVERTED = !SHOOTER_LEAD_INVERTED;
    
    //Feed
    public static final double FEED_ENC_POS_CONFIG = 1;
    public static final double FEED_ENC_VEL_CONFIG = 1;
    public static final boolean FEED_INVERTED = true;
    public static final IdleMode FEED_IDLE_MODE = IdleMode.kCoast;
    public static final int FEED_CURRENT_LIMIT = 40;

    //Indexer
    public static final double INDEXER_ENC_VEL_CONFIG = 1;
    public static final double INDEXER_ENC_POS_CONFIG = 1;
    public static final boolean INDEXER_INVERTED = true;
    public static final IdleMode INDEXER_IDLE_MODE = IdleMode.kCoast;
    public static final int INDEXER_CURRENT_LIMIT = 60;
   
    //Hood
    public static final boolean HOOD_INVERTED = false;
    public static final IdleMode HOOD_IDLE_MODE = IdleMode.kCoast;
    public static final int HOOD_CURRENT_LIMIT = 20;
    public static final double HOOD_ENC_POS_CONFIG = 1;
    public static final double HOOD_ENC_VEL_CONFIG = 1;
    public static final double HOOD_KV = 0;
    public static final double HOOD_KS = 0;
    public static final double HOOD_KA = 0;
    
    //Turret
    public static final boolean TURRET_INVERTED = true;
    public static final IdleMode TURRET_IDLE_MODE = IdleMode.kCoast;
    public static final int TURRET_CURRENT_LIMIT = 20;
    public static final double TURRET_KV = 0;
    public static final double TURRET_KA = 0;
    public static final double TURRET_KS = 0;
    public static final double TURRET_PID_TOLERANCE = Math.toRadians(8);//Radians
    public static final double TURRET_ENC_POS_CONFIG = 1;
    public static final double TURRET_ENC_VEL_CONFIG = 1;
    
    public static final double SPEED_TOLERENCE = 100;
    public static final double FEED_SPEED = 1;
    public static final double INDEX_SPEED = 0.35;
    public static final double TURRET_TOLERANCE = 0;
    
    //Target Translations
    public static final Translation2d RED_HUB_TARGET_POSE = new Translation2d(11.91,4);
    public static final Translation2d BLUE_HUB_TARGET_POSE = new Translation2d(4.54,4);
    public static final Translation2d RED_LEFT = new Translation2d(12.5,5.65);
    public static final Translation2d RED_RIGHT = new Translation2d(12.5,2.5);
    public static final Translation2d BLUE_LEFT = new Translation2d(4,5.65);
    public static final Translation2d BLUE_RIGHT = new Translation2d(4,2.5);
    public static final double MIDDLE_RED_TRENCHES = 11.9;
    public static final double MIDDLE_BLUE_TRENCHES = 5.8;
    public static final double LOW_RED_TRENCHES = 11.3;
    public static final double HIGH_RED_TRENCHES = 12.5;
    public static final double LOW_BLUE_TRENCHES = 3.9;
    public static final double HIGH_BLUE_TRENCHES = 5.3;

    public static final double TURRET_MAX_ANGLE = 4.8;
    public static final double TURRET_MIN_ANGLE = 0.9;
    public static final double HOOD_MAX_ANGLE = Math.toRadians(44);
    public static final double HOOD_MIN_ANGLE = Math.toRadians(12);
    public static final double HOOD_ROT_TO_RADIANS = 0.70686;
    public static final double TARGET_FEED_SPEED = 1;
    public static final double TARGET_INDEXER_SPEED = 1;
    public static final double INTAKE_AGITATION_SPEED = 0.5;
    
    public static final double FEED_LASER_THRESHOLD = 0.5;
    public static final double HOPPER_LASER_THRESHOLD = 0.5;
    
    public static final double TURRET_CHANGE = 8;//Defines how much change we should see in on 20 ms loop
    public static final double PIVOT_CHANGE = 2;

    public static final double HOOD_TRENCH_ANG = 12;
    public static final double FEED_ON_THRESHOLD = 2500;
    public static final double HOOD_ENCODER_FULL_ROTATION = 9*(Math.PI)/40;
    public static final double HOOD_ANGLE_OFFSET = Math.toRadians(12);
}
