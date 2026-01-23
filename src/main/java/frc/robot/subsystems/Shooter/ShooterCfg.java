package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class ShooterCfg {
    //Shooter Motor Config
    public static final int LEAD_SHOOTER_MOTOR_ID = 1;
    public static final int FOLLOWER_SHOOTER_MOTOR_ID = 2;
    public static final int HOOD_MOTOR_ID = 6;
    public static final int FEED_MOTOR_ID = 7;
    public static final int INDEXER_MOTOR_ID = 12;
    public static final int TURRET_MOTOR_ID = 13;

    public static final SparkFlex LEAD_SHOOTER_MOTOR = new SparkFlex(LEAD_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex FOLLOWER_SHOOTER_MOTOR = new SparkFlex(FOLLOWER_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax HOOD_MOTOR = new SparkMax(HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkFlex FEED_MOTOR = new SparkFlex(FEED_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax TURRET_MOTOR = new SparkMax(TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
}
