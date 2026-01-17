package frc.robot.subsystems.SwerveDrive;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

final class ChassisMotorCfg {
  //User defined Configs
  //drive
  public static final int DRIVE_FRONT_LEFT_ID = 17;
  public static final int DRIVE_FRONT_RIGHT_ID = 11;
  public static final int DRIVE_BACK_RIGHT_ID = 9;
  public static final int DRIVE_BACK_LEFT_ID = 5;

  public static final boolean DRIVE_FRONT_LEFT_REVERSED   = true;
  public static final boolean DRIVE_FRONT_RIGHT_REVERSED  = true;
  public static final boolean DRIVE_BACK_LEFT_REVERSED    = true;
  public static final boolean DRIVE_BACK_RIGHT_REVERSED   = true;
    
  //turn
  public static final int ANGLE_FRONT_LEFT_ID = 16;
  public static final int ANGLE_FRONT_RIGHT_ID = 10;
  public static final int ANGLE_BACK_RIGHT_ID = 8;
  public static final int ANGLE_BACK_LEFT_ID = 4;

  public static final boolean ANGLE_FRONT_LEFT_REVERSED   = true;
  public static final boolean ANGLE_FRONT_RIGHT_REVERSED  = true;
  public static final boolean ANGLE_BACK_RIGHT_REVERSED   = true;
  public static final boolean ANGLE_BACK_LEFT_REVERSED    = true;

  //Other configs
  public static final SparkFlex DRIVE_FRONT_LEFT   = new SparkFlex(DRIVE_FRONT_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex DRIVE_FRONT_RIGHT  = new SparkFlex(DRIVE_FRONT_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex DRIVE_BACK_RIGHT   = new SparkFlex(DRIVE_BACK_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex DRIVE_BACK_LEFT    = new SparkFlex(DRIVE_BACK_LEFT_ID, SparkLowLevel.MotorType.kBrushless);

  public static final boolean DRIVE_MOTOR_REVERSED[] = {
    DRIVE_FRONT_LEFT_REVERSED,
    DRIVE_FRONT_RIGHT_REVERSED,
    DRIVE_BACK_LEFT_REVERSED,
    DRIVE_BACK_RIGHT_REVERSED
  };

  public static final SparkFlex ANGLE_FRONT_LEFT   = new SparkFlex(ANGLE_FRONT_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex ANGLE_FRONT_RIGHT  = new SparkFlex(ANGLE_FRONT_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex ANGLE_BACK_RIGHT   = new SparkFlex(ANGLE_BACK_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex ANGLE_BACK_LEFT    = new SparkFlex(ANGLE_BACK_LEFT_ID, SparkLowLevel.MotorType.kBrushless);

  public static final boolean ANGLE_MOTOR_REVERSED[] = {
    ANGLE_FRONT_LEFT_REVERSED,
    ANGLE_FRONT_RIGHT_REVERSED,
    ANGLE_BACK_LEFT_REVERSED,
    ANGLE_BACK_RIGHT_REVERSED
  };

  public static final double DRIVE_FRONT_LEFT_KV  = 2.0286;
  public static final double DRIVE_FRONT_RIGHT_KV = 2.0286;
  public static final double DRIVE_BACK_LEFT_KV   = 2.0286;
  public static final double DRIVE_BACK_RIGHT_KV  = 2.0286;

  public static final double DRIVE_FRONT_LEFT_KS  = 0.18994;
  public static final double DRIVE_FRONT_RIGHT_KS = 0.18994;
  public static final double DRIVE_BACK_LEFT_KS   = 0.18994;
  public static final double DRIVE_BACK_RIGHT_KS  = 0.18994;

  public static final double DRIVE_FRONT_LEFT_KA  = 0.24283;
  public static final double DRIVE_FRONT_RIGHT_KA = 0.24283;
  public static final double DRIVE_BACK_LEFT_KA   = 0.24283;
  public static final double DRIVE_BACK_RIGHT_KA  = 0.24283;

  public static final double DRIVE_MOTOR_KV[] = { 
    DRIVE_FRONT_LEFT_KV,
    DRIVE_FRONT_RIGHT_KV,
    DRIVE_BACK_LEFT_KV,
    DRIVE_BACK_RIGHT_KV
  };

  public static final double DRIVE_MOTOR_KS[] = { 
    DRIVE_FRONT_LEFT_KS,
    DRIVE_FRONT_RIGHT_KS,
    DRIVE_BACK_LEFT_KS,
    DRIVE_BACK_RIGHT_KS
  };

  public static final double DRIVE_MOTOR_KA[] = { 
    DRIVE_FRONT_LEFT_KA,
    DRIVE_FRONT_RIGHT_KA,
    DRIVE_BACK_LEFT_KA,
    DRIVE_BACK_RIGHT_KA
  };
}