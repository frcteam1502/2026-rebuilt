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
  //TODO - Create RelativeEncoder for all motors

  //CANCoder objects for turret and hood encoders
  private final CANcoder turretAbsEncoder = ShooterCfg.TURRET_ABS_ENCODER;
  //TODO - Create CANcoder for hood absolute encoder

  //REV PIDF control objects
  SparkClosedLoopController shooterPIDController;

  //WPI PID control objects
  private final PIDController turretPIDController = new PIDController(ShooterCfg.TURRET_P_GAIN,
                                                                      ShooterCfg.TURRET_I_GAIN,
                                                                      ShooterCfg.TURRET_D_GAIN);

  //TODO - Creat WPI PID for hood

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
}
