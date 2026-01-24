package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

    public static final SparkFlex LEAD_SHOOTER_MOTOR = new SparkFlex(LEAD_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex FOLLOWER_SHOOTER_MOTOR = new SparkFlex(FOLLOWER_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax HOOD_MOTOR = new SparkMax(HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex FEED_MOTOR = new SparkFlex(FEED_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex INDEXER_MOTOR = new SparkFlex(INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax TURRET_MOTOR = new SparkMax(TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final CANcoder TURRET_ABS_ENCODER = new CANcoder(TURRET_ABS_ENCODER_ID);
    
    //PIDF Gains
    public static final double SHOOTER_P_GAIN = 0;
    public static final double SHOOTER_I_GAIN = 0;
    public static final double SHOOTER_D_GAIN = 0;

    public static final double TURRET_P_GAIN = 0;
    public static final double TURRET_I_GAIN = 0;
    public static final double TURRET_D_GAIN = 0;


    public static final double SHOOTER_ENC_POS_CONFIG = 1;
    public static final double SHOOTER_ENC_VEL_CONFIG = 1;
    public static final double SHOOTER_KV = 0;
    public static final double SHOOTER_KA = 0;
    public static final double SHOOTER_KS = 0;

    public static final boolean SHOOTER_LEAD_INVERTED = false;
    public static final IdleMode SHOOTER_IDLE_MODE = IdleMode.kCoast;
    public static final int SHOOTER_CURRENT_LIMIT = 40;
    public static final boolean SHOOTER_FOLLOW_INVERTED = !SHOOTER_LEAD_INVERTED;
    

    
}
