// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkFlex leadShooterMotor = ShooterCfg.LEAD_SHOOTER_MOTOR;
  private final SparkFlex followerShooterMotor = ShooterCfg.FOLLOWER_SHOOTER_MOTOR;
  private final SparkMax hoodMotor = ShooterCfg.HOOD_MOTOR;
  private final SparkFlex feedMotor = ShooterCfg.FEED_MOTOR;
  private final SparkFlex indexerMotor = ShooterCfg.INDEXER_MOTOR;
  private final SparkMax turretMotor = ShooterCfg.TURRET_MOTOR;

  //RelativeEncoder for all motors
  RelativeEncoder shooterLeadEncoder;
  RelativeEncoder shooterFollowerEncoder;
  RelativeEncoder hoodEncoder;
  RelativeEncoder feedEncoder;
  RelativeEncoder turretEncoder;
  RelativeEncoder indexerEncoder;

  //CANCoder objects for turret and hood encoders
  private final CANcoder turretAbsEncoder = ShooterCfg.TURRET_ABS_ENCODER;
  private final CANcoder hoodAbsEncoder = ShooterCfg.HOOD_ABS_ENCODER;
  
  //REV PIDF control objects
  SparkClosedLoopController shooterPIDController;

  //WPI PID control objects
  private final PIDController turretPIDController = new PIDController(ShooterCfg.TURRET_P_GAIN,
                                                                      ShooterCfg.TURRET_I_GAIN,
                                                                      ShooterCfg.TURRET_D_GAIN);
  
  private final PIDController hoodPIDController = new PIDController(ShooterCfg.HOOD_P_GAIN,
                                                                      ShooterCfg.HOOD_I_GAIN,
                                                                      ShooterCfg.HOOD_D_GAIN);  


  //Shooter lead motor config - REV closed loop speed control
  private final SparkFlexConfig shooterLeadConfig = new SparkFlexConfig();
  private final EncoderConfig shooterLeadEncoderConfig = new EncoderConfig();
  private final ClosedLoopConfig shooterLeadPIDFConfig = new ClosedLoopConfig();
  private final FeedForwardConfig shooterLeadFFConfig = new FeedForwardConfig();

  //Shooter follower motor config - follow lead motor
  private final SparkFlexConfig shooterFollowerConfig = new SparkFlexConfig();
  private final EncoderConfig shooterFollowerEncoderConfig = new EncoderConfig();

  //Hood motor config - WPI position control 
  private final SparkFlexConfig shooterHoodConfig = new SparkFlexConfig();
  private final FeedForwardConfig shooterHoodFFConfig = new FeedForwardConfig();
  
  //Feed motor - open loop
  private final EncoderConfig shooterFeedEncoderConfig = new EncoderConfig();
  private final SparkFlexConfig shooterFeedConfig = new SparkFlexConfig();

  //Index motor - open loop
  private final EncoderConfig shooterIndexerEncoderConfig = new EncoderConfig();
  private final SparkFlexConfig shooterIndexerConfig = new SparkFlexConfig();

  //Turret motor - WPI position control
  private final SparkFlexConfig shooterTurretConfig = new SparkFlexConfig();
  private final FeedForwardConfig shooterTurretFFConfig = new FeedForwardConfig();
  

  public Shooter() {
    configShooterMotors();
    configFeedMotor();
    configHoodMotor();
    configIndexMotor();
    configTurretMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }
  private void configShooterMotors() {
    //Config the encoders
    shooterLeadEncoder = leadShooterMotor.getEncoder();
    shooterLeadEncoderConfig.positionConversionFactor(ShooterCfg.SHOOTER_ENC_POS_CONFIG);
    shooterLeadEncoderConfig.velocityConversionFactor(ShooterCfg.SHOOTER_ENC_VEL_CONFIG);

    //Config PID values
    shooterPIDController = leadShooterMotor.getClosedLoopController();
    shooterLeadPIDFConfig.p(ShooterCfg.SHOOTER_P_GAIN);
    shooterLeadPIDFConfig.i(ShooterCfg.SHOOTER_I_GAIN);
    shooterLeadPIDFConfig.d(ShooterCfg.SHOOTER_D_GAIN);

    //Config FF values
    shooterLeadFFConfig.kV(ShooterCfg.SHOOTER_KV);
    shooterLeadFFConfig.kA(ShooterCfg.SHOOTER_KA);
    shooterLeadFFConfig.kS(ShooterCfg.SHOOTER_KS);
    
    shooterLeadPIDFConfig.apply(shooterLeadFFConfig);

    //Config SparkFlex
    shooterLeadConfig.inverted(ShooterCfg.SHOOTER_LEAD_INVERTED);
    shooterLeadConfig.idleMode(ShooterCfg.SHOOTER_IDLE_MODE);
    shooterLeadConfig.smartCurrentLimit(ShooterCfg.SHOOTER_CURRENT_LIMIT);

    //Apply Encoder and Closed Loop Configs to the SparkFlexCfg
    shooterLeadConfig.apply(shooterLeadEncoderConfig);
    shooterLeadConfig.apply(shooterLeadPIDFConfig);

    //Write to the SparkFlex
    leadShooterMotor.configure(shooterLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Setup the follower motor
    //Config the encoders
    shooterFollowerEncoderConfig.positionConversionFactor(ShooterCfg.SHOOTER_ENC_POS_CONFIG);
    shooterFollowerEncoderConfig.velocityConversionFactor(ShooterCfg.SHOOTER_ENC_VEL_CONFIG);

    //Config Spark Flex
    shooterFollowerConfig.follow(ShooterCfg.LEAD_SHOOTER_MOTOR_ID);
    shooterFollowerConfig.inverted(ShooterCfg.SHOOTER_FOLLOW_INVERTED);
    shooterFollowerConfig.idleMode(ShooterCfg.SHOOTER_IDLE_MODE);
    shooterFollowerConfig.smartCurrentLimit(ShooterCfg.SHOOTER_CURRENT_LIMIT);

    shooterFollowerConfig.apply(shooterLeadEncoderConfig);

    //Write to the SparkFlex
    followerShooterMotor.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Hood Motor COnfig
  private void configHoodMotor() {
    //Config SparkFlex
    shooterHoodConfig.inverted(ShooterCfg.SHOOTER_HOOD_INVERTED);
    shooterHoodConfig.idleMode(ShooterCfg.SHOOTER_HOOD_IDLE_MODE);
    shooterHoodConfig.smartCurrentLimit(ShooterCfg.SHOOTER_HOOD_CURRENT_LIMIT);

    //Config FF values
    shooterHoodFFConfig.kV(ShooterCfg.SHOOTER_HOOD_KV);
    shooterHoodFFConfig.kA(ShooterCfg.SHOOTER_HOOD_KA);
    shooterHoodFFConfig.kS(ShooterCfg.SHOOTER_HOOD_KS);

    //Write to the SparkFlex
    hoodMotor.configure(shooterHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Turret motor config
private void configTurretMotor() {
    //Config SparkFlex
    shooterTurretConfig.inverted(ShooterCfg.SHOOTER_TURRET_INVERTED);
    shooterTurretConfig.idleMode(ShooterCfg.SHOOTER_TURRET_IDLE_MODE);
    shooterTurretConfig.smartCurrentLimit(ShooterCfg.SHOOTER_TURRET_CURRENT_LIMIT);

    //Config FF values
    shooterTurretFFConfig.kV(ShooterCfg.SHOOTER_TURRET_KV);
    shooterTurretFFConfig.kA(ShooterCfg.SHOOTER_TURRET_KA);
    shooterTurretFFConfig.kS(ShooterCfg.SHOOTER_TURRET_KS);

    //Write to the SparkFlex
    turretMotor.configure(shooterTurretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Index Motor Config
  private void configIndexMotor() {
    //Config the encoders
    indexerEncoder = feedMotor.getEncoder();
    shooterIndexerEncoderConfig.positionConversionFactor(ShooterCfg.INDEXER_ENC_POS_CONFIG);
    shooterIndexerEncoderConfig.velocityConversionFactor(ShooterCfg.INDEXER_ENC_VEL_CONFIG);

    //Config SparkFlex
    shooterIndexerConfig.inverted(ShooterCfg.INDEXER_INVERTED);
    shooterIndexerConfig.idleMode(ShooterCfg.INDEXER_IDLE_MODE);
    shooterIndexerConfig.smartCurrentLimit(ShooterCfg.INDEXER_CURRENT_LIMIT);

    //Write to the SparkFlex
    indexerMotor.configure(shooterIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  //Feed Motor
  private void configFeedMotor() {
    //Config the encoders
    feedEncoder = feedMotor.getEncoder();
    shooterFeedEncoderConfig.positionConversionFactor(ShooterCfg.FEED_ENC_POS_CONFIG);
    shooterFeedEncoderConfig.velocityConversionFactor(ShooterCfg.FEED_ENC_VEL_CONFIG);

    //Config SparkFlex
    shooterFeedConfig.inverted(ShooterCfg.FEED_INVERTED);
    shooterFeedConfig.idleMode(ShooterCfg.FEED_IDLE_MODE);
    shooterFeedConfig.smartCurrentLimit(ShooterCfg.FEED_CURRENT_LIMIT);

    //Write to the SparkFlex
    feedMotor.configure(shooterFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
