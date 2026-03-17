// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Logger;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkFlex leadShooterMotor = ShooterCfg.LEAD_SHOOTER_MOTOR;
  private final SparkFlex followerShooterMotor = ShooterCfg.FOLLOWER_SHOOTER_MOTOR;
  private final SparkMax hoodMotor = ShooterCfg.HOOD_MOTOR;
  private final SparkMax feedMotor = ShooterCfg.FEED_MOTOR;
  private final SparkFlex indexerMotor = ShooterCfg.INDEXER_MOTOR;
  private final SparkMax turretMotor = ShooterCfg.TURRET_MOTOR;
  
  private LaserCan feedLaser;
  private LaserCan hopperLaser;

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
  private final SparkMaxConfig shooterHoodConfig = new SparkMaxConfig();
  private final EncoderConfig shooterHoodEncoderConfig = new EncoderConfig();
  private final CANcoderConfiguration hoodCANcoderConfig = new CANcoderConfiguration();
  
  //Feed motor - open loop
  private final EncoderConfig shooterFeedEncoderConfig = new EncoderConfig();
  private final SparkFlexConfig shooterFeedConfig = new SparkFlexConfig();

  //Index motor - open loop
  private final EncoderConfig shooterIndexerEncoderConfig = new EncoderConfig();
  private final SparkFlexConfig shooterIndexerConfig = new SparkFlexConfig();

  //Turret motor - WPI position control
  private final SparkMaxConfig shooterTurretConfig = new SparkMaxConfig();
  private final EncoderConfig shooterTurretEncoderConfig = new EncoderConfig();
  private final CANcoderConfiguration turretCANcoderConfig = new CANcoderConfiguration();

  private double shooterSetSpeed = 0;
  private double angleToTarget;
  private double turretSetAngle = Math.toRadians(180);
  private double hoodSetAngle = Math.toRadians(35);
  private Translation2d targetTranslation = new Translation2d(0,0);

  private boolean isTestMode = false;

  private enum ShooterState{
    OFF,
    WAIT,
    READY,
    STARTFEED,
    SHOOTING,
    SPIN_UP;
  }

  private enum TurretState{
    INACTIVE,
    MOVE_TO_TARGET,
    ON_TARGET;
  }

   private enum IndexerState{
    OFF,
    WAIT,
    ON;
  }

  private ShooterState shooterState = ShooterState.WAIT;
   
  private TurretState turretState = TurretState.MOVE_TO_TARGET;
  
  private IndexerState indexerState = IndexerState.OFF;

  private DriveSubsystem drive;
  
  private Intake intake;

  private boolean autoAimToggle = true;

  private boolean autoHoodToggle = true;

  //Create a SysIdRoutine object for characterizing the Shooter
  private final SysIdRoutine sysIdShooter = 
  new SysIdRoutine(
    //Create a new SysID Congig with default ramp rate (0.1 V/s), step (7V), and time out values
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      voltage -> {
        setSysIDVoltage(voltage);},
      // Tell SysId how to record a frame of data for each motor on the mechanism being
      // characterized.
      log -> {
        //Log a frame for the frontLeft Motor
        log.motor("shooter")
          .voltage(getShooterMotorVoltage())
          .angularPosition(getShooterAngularPositionRadians())
          .angularVelocity(getShooterAngularVelocityRadiansPerSec());
      },
      // Tell SysId to make generated commands require this subsystem, suffix test state in
      // WPILog with this subsystem's name ("drive")
      this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return sysIdShooter.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return sysIdShooter.dynamic(direction);
  }

  public Shooter(DriveSubsystem drive, Intake intake) {
    this.drive = drive;
    this.intake = intake;

    SmartDashboard.putNumber("Shooter Test Speed", shooterSetSpeed);
    SmartDashboard.putNumber("Hood Test Angle", Math.toDegrees(hoodSetAngle));

    configShooterMotors();
    configFeedMotor();
    configHoodMotor();
    configIndexMotor();
    configTurretMotor();

    registerLoggerObjects();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShooterState();
    //updateTurretState();
    updateHoodAngleSetPoint();
    updateShooterSetPoint();
    //updateTurretAngleSetPoint();
    updateDashboard();
    isShootingReady();
  }

  private void registerLoggerObjects(){
    Logger.RegisterSparkFlex("Shooter Lead", ShooterCfg.LEAD_SHOOTER_MOTOR);
    Logger.RegisterSparkFlex("Shooter Follower", ShooterCfg.FOLLOWER_SHOOTER_MOTOR);
    Logger.RegisterSparkFlex("Indexer",ShooterCfg.INDEXER_MOTOR);

    Logger.RegisterSparkMax("Hood", ShooterCfg.HOOD_MOTOR);
    Logger.RegisterSparkMax("Turret", ShooterCfg.TURRET_MOTOR);
    Logger.RegisterSparkMax("Hood", ShooterCfg.FEED_MOTOR);

    Logger.RegisterCanCoder("Hood Abs Encoder", ShooterCfg.HOOD_ABS_ENCODER);
    Logger.RegisterCanCoder("Hood Abs Encoder", ShooterCfg.HOOD_ABS_ENCODER);
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
    shooterFollowerConfig.follow(ShooterCfg.LEAD_SHOOTER_MOTOR_ID,ShooterCfg.SHOOTER_FOLLOW_INVERTED);
    shooterFollowerConfig.idleMode(ShooterCfg.SHOOTER_IDLE_MODE);
    shooterFollowerConfig.smartCurrentLimit(ShooterCfg.SHOOTER_CURRENT_LIMIT);

    shooterFollowerConfig.apply(shooterFollowerEncoderConfig);

    //Write to the SparkFlex
    followerShooterMotor.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Hood Motor COnfig
  private void configHoodMotor() {
    //Config the encoders
    shooterHoodEncoderConfig.positionConversionFactor(ShooterCfg.HOOD_ENC_POS_CONFIG);
    shooterHoodEncoderConfig.velocityConversionFactor(ShooterCfg.HOOD_ENC_VEL_CONFIG);

    //Config SparkFlex
    shooterHoodConfig.inverted(ShooterCfg.HOOD_INVERTED);
    shooterHoodConfig.idleMode(ShooterCfg.HOOD_IDLE_MODE);
    shooterHoodConfig.smartCurrentLimit(ShooterCfg.HOOD_CURRENT_LIMIT);

    shooterHoodConfig.apply(shooterHoodEncoderConfig);

    //Write to the SparkFlex
    hoodMotor.configure(shooterHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Set absolute encoder magnet configuration
    hoodCANcoderConfig.MagnetSensor.MagnetOffset = ShooterCfg.HOOD_ABS_ENCODER_OFFSET;
    hoodCANcoderConfig.MagnetSensor.SensorDirection = ShooterCfg.HOOD_CAN_CODER_DIRECTION;
    hoodCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ShooterCfg.DISCONTINUITY_POINT;
    hoodAbsEncoder.getConfigurator().apply(hoodCANcoderConfig);

  }

  //Turret motor config
private void configTurretMotor() {
 //Config the encoders
    shooterTurretEncoderConfig.positionConversionFactor(ShooterCfg.TURRET_ENC_POS_CONFIG);
    shooterTurretEncoderConfig.velocityConversionFactor(ShooterCfg.TURRET_ENC_VEL_CONFIG);


    //Config SparkFlex
    shooterTurretConfig.inverted(ShooterCfg.TURRET_INVERTED);
    shooterTurretConfig.idleMode(ShooterCfg.TURRET_IDLE_MODE);
    shooterTurretConfig.smartCurrentLimit(ShooterCfg.TURRET_CURRENT_LIMIT);

    shooterTurretConfig.apply(shooterTurretEncoderConfig);

    //Write to the SparkFlex
    turretMotor.configure(shooterTurretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Set absolute encoder magnet configuration
    turretCANcoderConfig.MagnetSensor.MagnetOffset = ShooterCfg.TURRET_ABS_ENCODER_OFFSET;
    turretCANcoderConfig.MagnetSensor.SensorDirection = ShooterCfg.TURRET_CAN_CODER_DIRECTION;
    turretCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ShooterCfg.DISCONTINUITY_POINT;
    turretAbsEncoder.getConfigurator().apply(turretCANcoderConfig);

    turretPIDController.setTolerance(ShooterCfg.TURRET_PID_TOLERANCE);
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

    shooterIndexerConfig.apply(shooterIndexerEncoderConfig);

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

    shooterFeedConfig.apply(shooterFeedEncoderConfig);

    //Write to the SparkFlex
    feedMotor.configure(shooterFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getShooterLeadPos(){
    return shooterLeadEncoder.getPosition();
  }
   public double getShooterLeadVel(){
    return shooterLeadEncoder.getVelocity();
  }

  public double getShooterFollowPos(){
    return shooterFollowerEncoder.getPosition();
  }
  public double getShooterFollowVel(){
    return shooterFollowerEncoder.getVelocity();
  }
  

  public double getIndexPos(){
    return indexerEncoder.getPosition();
  }
   public double getIndexVel(){
    return indexerEncoder.getVelocity();
  }
  public void setIndexSpeed(double speed){
    indexerMotor.set(speed);
  }

  public double getFeedPos(){
    return feedEncoder.getPosition();
  }
  public double getFeedVel(){
    return feedEncoder.getVelocity();
  }
  public void setFeedSpeed(double speed){
    feedMotor.set(speed);
  }

  public double getTurretPos(){
    return turretEncoder.getPosition();
  }
  public double getTurretVel(){
    return turretEncoder.getVelocity();
  }
  
  public void setTurretForward(){
    turretMotor.set(1);
  }
  public void setTurretReverse(){
    turretMotor.set(-1);
  }
  public void setTurretOff(){
    turretMotor.set(0);
  }
  
  public double getTurretAbsPositionZeroed() {
    //CANcoders in Phoenix return rotations 0 to 1
    var angle = turretAbsEncoder.getAbsolutePosition();
    return angle.getValueAsDouble()*2.0*Math.PI;
  }

  public double getTurretAbsVelocity() {
    //CANcoders in Phoenix return rotations 0 to 1
    var velocity = turretAbsEncoder.getVelocity();
    return velocity.getValueAsDouble()*2.0*Math.PI;
  }

  public double getHoodAbsPositionZeroed() {
    //CANcoders in Phoenix return rotations 0 to 1
    var angle = hoodAbsEncoder.getAbsolutePosition();
    return (angle.getValueAsDouble()*ShooterCfg.HOOD_ROT_TO_RADIANS)+ShooterCfg.HOOD_ANGLE_OFFSET;
  }

  public void setShooterSpeed(double speed){
    shooterSetSpeed = speed;
  }

  public void setTurretAngle(double angle){
    turretSetAngle = angle;
  }

  public void setHoodAngle(double angleDeg){
    hoodSetAngle = Math.toRadians(angleDeg);
  }

  private void updateDashboard(){
    SmartDashboard.putNumber("Turret Angle", Math.toDegrees(getTurretAbsPositionZeroed()));
    SmartDashboard.putNumber("Turret Velocity", getTurretAbsVelocity());
    SmartDashboard.putNumber("Turret Motor Command", turretMotor.getAppliedOutput());
    SmartDashboard.putString("Turret State", turretState.toString());
    SmartDashboard.putNumber("Turret Set Angle", Math.toDegrees(turretSetAngle));
    SmartDashboard.putBoolean("Is Turret At Set Point", turretPIDController.atSetpoint());
    SmartDashboard.putNumber("Target Translation X", targetTranslation.getX());
    SmartDashboard.putNumber("Target Translation Y", targetTranslation.getY());
    SmartDashboard.putNumber("Angle to Target", calculateTargetAngle(targetTranslation));
    SmartDashboard.putNumber("Distance To Target", calculateTargetDistance(targetTranslation));
    SmartDashboard.putNumber("Target Shooter Speed", shooterSetSpeed);
    SmartDashboard.putString("Indexer State", indexerState.toString());
    SmartDashboard.putNumber("Feed Speed", getFeedVel());
    SmartDashboard.putNumber("Indexer Speed", getIndexVel());
    SmartDashboard.putBoolean("Auto Aim Toggle", autoAimToggle);
    SmartDashboard.putNumber("Shooter Speed (RPM)",shooterLeadEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Output", leadShooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Lead Current",leadShooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Follow Current",followerShooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Actual Hood Angle", Math.toDegrees(getHoodAbsPositionZeroed()));
    SmartDashboard.putNumber("Target Hood Angle", Math.toDegrees(hoodSetAngle));
    SmartDashboard.putNumber("Hood Command", hoodMotor.getAppliedOutput());
    SmartDashboard.putString("Shooter State", shooterState.toString());
    SmartDashboard.putBoolean("Shooter At Setpoint", isShooterAtSetPoint());
    SmartDashboard.putBoolean("Hood At Setpoint", hoodPIDController.atSetpoint());
    SmartDashboard.putBoolean("Is Shooting Ready", isShootingReady());
    SmartDashboard.putBoolean("Is Shooter Test Active", isTestMode);
  }

  public void updateShooterSetPoint(){
    shooterPIDController.setSetpoint(shooterSetSpeed, SparkFlex.ControlType.kVelocity);
  }

  public void updateHoodAngleSetPoint(){
    var hoodCommand = hoodPIDController.calculate(getHoodAbsPositionZeroed(), hoodSetAngle);
    hoodMotor.setVoltage(hoodCommand);
  }

  public void updateTurretAngleSetPoint(){
    var turretCommand = turretPIDController.calculate(getTurretAbsPositionZeroed(), turretSetAngle);
    turretMotor.setVoltage(turretCommand);
  }
  private void updateShooterState(){
    switch(shooterState){
      case OFF:
        //DO NOTHING
        break;
      
        case WAIT:
        if(isTestMode){
          hoodSetAngle = Math.toRadians(SmartDashboard.getNumber("Hood Test Angle", ShooterCfg.HOOD_MIN_ANGLE));
        }else{
          if (autoHoodToggle){
            //hoodSetAngle = lookupHoodAngle(targetTranslation);
            hoodSetAngle = Math.toRadians(15);
          }else{
            //DO NOT UPDATE
          }
        }

        if(hoodPIDController.atSetpoint() &&
           turretState == TurretState.ON_TARGET){
           shooterState = ShooterState.READY;
        }else{
          //DO NOTHING
        }
        break;

      case READY:
        if(isTestMode){
          hoodSetAngle = Math.toRadians(SmartDashboard.getNumber("Hood Test Angle", ShooterCfg.HOOD_MIN_ANGLE));
        }else{
          if (autoHoodToggle){
            //hoodSetAngle = lookupHoodAngle(targetTranslation);
            hoodSetAngle = Math.toRadians(15);
          }else{
            //DO NOT UPDATE
          }
        }

        if(!hoodPIDController.atSetpoint() ||
           turretState != TurretState.ON_TARGET){
            shooterState = ShooterState.WAIT;
        }else{
          //DO NOTHING
        }
        break;

      case STARTFEED:
        if(isTestMode){
          shooterSetSpeed = SmartDashboard.getNumber("Shooter Test Speed", 0.0);
          hoodSetAngle = Math.toRadians(SmartDashboard.getNumber("Hood Test Angle", ShooterCfg.HOOD_MIN_ANGLE));
        }else{
          shooterSetSpeed = lookupShooterSpeed(targetTranslation);
          setFeedSpeed(ShooterCfg.FEED_SPEED);
          
          if (autoHoodToggle){
            hoodSetAngle = lookupHoodAngle(targetTranslation);
          }else{
            //DO NOT UPDATE
          }
        }

        if(isShooterAtSetPoint()                    &&
           hoodPIDController.atSetpoint()           &&
           turretState == TurretState.ON_TARGET     &&
           getFeedVel() >= ShooterCfg.FEED_ON_THRESHOLD){
            setIndexSpeed(ShooterCfg.INDEX_SPEED);
            intake.shooterRequestIntakeOn();
            shooterState = ShooterState.SHOOTING;
          }else {
            //NOTHING
        }
      break;

      case SHOOTING:
        if(isTestMode){
          shooterSetSpeed = SmartDashboard.getNumber("Shooter Test Speed", 0.0);
          hoodSetAngle = Math.toRadians(SmartDashboard.getNumber("Hood Test Angle", ShooterCfg.HOOD_MIN_ANGLE));
        }else{
          shooterSetSpeed = lookupShooterSpeed(targetTranslation);
          setFeedSpeed(ShooterCfg.FEED_SPEED);
          
          if (autoHoodToggle){
            hoodSetAngle = lookupHoodAngle(targetTranslation);
          }else{
            //DO NOT UPDATE
          }
        }

        if(intake.isHopperIn()){
          intake.shooterRequestIntakeOnSlow();
        }else{
          intake.shooterRequestIntakeOn();
        }
        
        if(!isShooterAtSetPoint()||
           !hoodPIDController.atSetpoint()     ||
           turretState != TurretState.ON_TARGET||
           getFeedVel() < ShooterCfg.FEED_ON_THRESHOLD){
            setIndexSpeed(0);
            intake.shooterRequestIntakeOff();
            shooterState = ShooterState.SPIN_UP;
          }else {
            setIndexSpeed(ShooterCfg.INDEX_SPEED);
        }
        break;

      case SPIN_UP:
        if(isTestMode){
          shooterSetSpeed = SmartDashboard.getNumber("Shooter Test Speed", 0.0);
          hoodSetAngle = Math.toRadians(SmartDashboard.getNumber("Hood Test Angle", ShooterCfg.HOOD_MIN_ANGLE));
        }else{
          shooterSetSpeed = lookupShooterSpeed(targetTranslation);
          setFeedSpeed(ShooterCfg.FEED_SPEED);
          
          if (autoHoodToggle){
            hoodSetAngle = lookupHoodAngle(targetTranslation);
          }else{
            //DO NOT UPDATE
          }
        }

        if(isShooterAtSetPoint() &&
           hoodPIDController.atSetpoint()      &&
           turretState == TurretState.ON_TARGET){
            setFeedSpeed(ShooterCfg.FEED_SPEED);
            shooterState = ShooterState.STARTFEED;
        }else{
          //DO NOTHING
      }
        break;
    } 
  }

  public void toggleTestMode(){
    if(isTestMode){
      isTestMode = false;
    }else{
      isTestMode = true;
    }
  }

  public void setShooterToWait(){
    setIndexSpeed(0);
    setFeedSpeed(0);
    intake.shooterRequestIntakeOff();
    shooterSetSpeed = 0;
    
    if(isTestMode){
          hoodSetAngle = Math.toRadians(SmartDashboard.getNumber("Hood Test Angle", ShooterCfg.HOOD_MIN_ANGLE));
        }else{
          if (autoHoodToggle){
            hoodSetAngle = lookupHoodAngle(targetTranslation);
          }else{
            //DO NOT UPDATE
          }
    }
      
    shooterState = ShooterState.WAIT;
  }

  public void setShooterOn(){
    shooterState = ShooterState.SPIN_UP;
  }

  public void setShooterOff(){
    shooterState = ShooterState.OFF;
    shooterSetSpeed = 0;
    setIndexSpeed(0);
    setFeedSpeed(0);
  }

  private void updateTurretState(){
    switch(turretState){
      case INACTIVE:
        //Do nothing here
        break;
      case MOVE_TO_TARGET:
        targetTranslation = calculateTargetPosition();
        turretSetAngle = calculateTargetAngle(targetTranslation);
        if(isTurretPointingAtTarget()){
          turretState = TurretState.ON_TARGET;
        } else{
          //Do nothing
        } 
        break;
      case ON_TARGET:
        targetTranslation = calculateTargetPosition();
        turretSetAngle = calculateTargetAngle(targetTranslation);
        if(!isTurretPointingAtTarget()){
          turretState = TurretState.MOVE_TO_TARGET;
        } else{
          //Do nothing
        } 
    }
  }

  private void setAutoAimOn(){
    turretState = TurretState.MOVE_TO_TARGET;
  }

  private void setAutoAimOff(){
    turretState = TurretState.INACTIVE;
  }

  public boolean isShooterAtSetPoint(){
    if(shooterLeadEncoder.getVelocity() <= shooterSetSpeed+ShooterCfg.SHOOTER_ALLOWED_ERROR &&
       shooterLeadEncoder.getVelocity() >= shooterSetSpeed-ShooterCfg.SHOOTER_ALLOWED_ERROR){
        return true;
      }else{
        return false;
    }   
  }

  public boolean isTurretAtSetpoint(){
    return turretPIDController.atSetpoint();
  }

  public boolean isHoodAtSetpoint(){
    return hoodPIDController.atSetpoint();
  }

  private boolean isTurretPointingAtTarget(){
    if(getTurretAbsPositionZeroed() <= angleToTarget+ShooterCfg.TURRET_PID_TOLERANCE &&
       getTurretAbsPositionZeroed() >= angleToTarget-ShooterCfg.TURRET_PID_TOLERANCE){
        return true;
       }else{
        return false;
       }
  }

  private double lookupShooterSpeed(Translation2d targetPose){
    //TODO Look UP shooter speed and set the shooterSetSpeed to the lookup value
    var distance = calculateTargetDistance(targetPose);
    int lookup_index = (int)(4*distance);

    //Make sure you are not indexing outside of the array
    if(lookup_index < 0){
      lookup_index = 0;
    }else if (lookup_index >= ShooterLookup.LOOKUP.length){
      lookup_index = ShooterLookup.LOOKUP.length - 1;
    }else{
      //Array size is inbounds
    }

    return ShooterLookup.LOOKUP[lookup_index][0];
    //CL - Was causing array out of bounds need to debug
  }
  private double lookupHoodAngle(Translation2d targetPose){
    //TODO Look UP Hood Angle and set the hoodAngle to the lookup value
    var distance = calculateTargetDistance(targetPose);
    var robotPose = drive.getEstimatedPose2d();
    int lookup_index = (int)(4*distance);

    //Make sure you are not indexing outside of the array
    if(lookup_index < 0){
      lookup_index = 0;
    }else if (lookup_index >= ShooterLookup.LOOKUP.length){
      lookup_index = ShooterLookup.LOOKUP.length - 1;
    }else{
      //Array size is inbounds
    }

    /*if((robotPose.getX() > ShooterCfg.LOW_RED_TRENCHES)&&
      (robotPose.getX() < ShooterCfg.HIGH_RED_TRENCHES)){
        return Math.toRadians(ShooterCfg.HOOD_TRENCH_ANG);
      }else if((robotPose.getX() > ShooterCfg.LOW_BLUE_TRENCHES)&&
      (robotPose.getX() < ShooterCfg.HIGH_BLUE_TRENCHES)){
        return Math.toRadians(ShooterCfg.HOOD_TRENCH_ANG);
      }else{
        return (Math.toRadians(ShooterLookup.LOOKUP[lookup_index][1]));
      }*/
    if(drive.isInTrenchZone()){
        return Math.toRadians(ShooterCfg.HOOD_TRENCH_ANG);
      }else{
        return (Math.toRadians(ShooterLookup.LOOKUP[lookup_index][1]));
      }
  }

  private double calculateTargetAngle(Translation2d targetPose){
    double angle;
    angleToTarget = drive.getDistanceAngleToPoint(targetPose).getY();

    if (angleToTarget < ShooterCfg.TURRET_MIN_ANGLE){
      angle = ShooterCfg.TURRET_MIN_ANGLE;
    }else if (angleToTarget > ShooterCfg.TURRET_MAX_ANGLE){
      angle = ShooterCfg.TURRET_MAX_ANGLE;
    }else{
      angle = angleToTarget;
    }
    return angle;
  }

  private double calculateTargetDistance(Translation2d targetPose){
    return drive.getDistanceAngleToPoint(targetPose).getX();
  }

  private Translation2d calculateTargetPosition(){
    var alliance = DriverStation.getAlliance();
    var robotPose = drive.getEstimatedPose2d();
    if (alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Red){
        if(robotPose.getX() >= 12.5){
          return ShooterCfg.RED_HUB_TARGET_POSE;
        }else if(robotPose.getY() >= 4.03){
          return ShooterCfg.RED_LEFT;
        }else{
          return ShooterCfg.RED_RIGHT;
        }
      }else{
        if(robotPose.getX() <= 4.54){
          return ShooterCfg.BLUE_HUB_TARGET_POSE;
        }else if(robotPose.getY() >= 4.03){
          return ShooterCfg.BLUE_LEFT;
        }else{
          return ShooterCfg.BLUE_RIGHT;
        }
      }
    }else{
      return ShooterCfg.BLUE_LEFT;
    }
  }
private void updateIndexerState(){
    switch (indexerState){
      case OFF:
      setFeedSpeed(0);
      setIndexSpeed(0);
        break;
      case WAIT:
        if (getFeedVel() == ShooterCfg.TARGET_FEED_SPEED){
          indexerState = IndexerState.ON; 
        }else{
          setIndexSpeed(0);
          intake.setIntakeSpeed(0);
        }
        break;
      case ON:
        if (getFeedVel() != ShooterCfg.TARGET_FEED_SPEED){
          indexerState = IndexerState.WAIT; 
        }else{
          setIndexSpeed(ShooterCfg.TARGET_INDEXER_SPEED);
          intake.setIntakeSpeed(ShooterCfg.INTAKE_AGITATION_SPEED);
        }
       break;
    }
  }
  public void setIndexerWaitCycleOn(){
    setFeedSpeed(ShooterCfg.TARGET_FEED_SPEED);
    indexerState = IndexerState.WAIT;
  }
  public void setIndexerWaitCycleOff(){
    indexerState = IndexerState.OFF;
  }
  public boolean isBallInFeeder(){
    LaserCan.Measurement measurement = feedLaser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      if (measurement.distance_mm < ShooterCfg.FEED_LASER_THRESHOLD){
        return true;
      }else{
        return false;
      }
    }else{
      return false;
    }
  }
  public boolean isBallInHopper(){
    LaserCan.Measurement hopperMeasurement = hopperLaser.getMeasurement();
    if (hopperMeasurement != null && hopperMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      if (hopperMeasurement.distance_mm < ShooterCfg.HOPPER_LASER_THRESHOLD){
        return true;
      }else{
        return false;
      }
    }else{
      return false;
    }
  }
  public void toggleAutoAim(){
    if(autoAimToggle == false){
      autoAimToggle = true;
      setAutoAimOn();
    }else{
      autoAimToggle = false;
      setAutoAimOff();
    }
  }
  public void toggleHoodAim(){
    if(autoHoodToggle == false){
      autoHoodToggle = true;
    }else{
      autoHoodToggle = false;
    }
  }

  public void moveTurretManually(double input){
    if(!autoAimToggle){
      double change = Math.signum(input) * ShooterCfg.TURRET_CHANGE;
      double newPosition = turretSetAngle + change;

      if(newPosition > ShooterCfg.TURRET_MAX_ANGLE){
        newPosition = ShooterCfg.TURRET_MAX_ANGLE;
      }else if (newPosition < ShooterCfg.TURRET_MIN_ANGLE){
        newPosition = ShooterCfg.TURRET_MIN_ANGLE;
      }else{
        //Do nothing, newPosition is in-bounds, allow the set position to get updated
      }
      turretSetAngle = newPosition;
    }
  }
  public void moveHoodManually(double input){
    if(!autoHoodToggle){
      double change = Math.signum(input) * ShooterCfg.TURRET_CHANGE;
      double newPosition = hoodSetAngle + change;

      if(newPosition > ShooterCfg.HOOD_MAX_ANGLE){
        newPosition = ShooterCfg.HOOD_MAX_ANGLE;
      }else if (newPosition < ShooterCfg.HOOD_MIN_ANGLE){
        newPosition = ShooterCfg.HOOD_MIN_ANGLE;
      }else{
        //Do nothing, newPosition is in-bounds, allow the set position to get updated
      }
      hoodSetAngle = newPosition;
    }
  }

  public void setSysIDVoltage(Voltage volts){
    //Set drive motor open-loop voltage
    leadShooterMotor.setVoltage(volts.magnitude());
  }

  public Voltage getShooterMotorVoltage(){
    var busVoltage = Voltage.ofBaseUnits(leadShooterMotor.getBusVoltage(), Volts);
    var appliedOutput = leadShooterMotor.getAppliedOutput();
    return (busVoltage.times(appliedOutput));
  }

  public Angle getShooterAngularPositionRadians(){
    return Angle.ofBaseUnits((shooterLeadEncoder.getPosition()*2*Math.PI), Radians);
  }

  public AngularVelocity getShooterAngularVelocityRadiansPerSec(){
    return AngularVelocity.ofBaseUnits((shooterLeadEncoder.getPosition()*2*Math.PI), RadiansPerSecond);
  }
  public void setFeedOn(){
    setFeedSpeed(1);
  }
  public void setFeedOff(){
    setFeedSpeed(0);
  }
  public void setIndexerOn(){
    setIndexSpeed(1);
  }
  public void setIndexerOff(){
    setIndexSpeed(0);
  }
  public boolean isShootingReady(){
    if (isShooterAtSetPoint()                     &&
        hoodPIDController.atSetpoint()            &&
        turretState == TurretState.ON_TARGET      &&
        getFeedVel() >= ShooterCfg.FEED_ON_THRESHOLD){
          return true;
        }
        else{
          return false;
        }
  }
}
