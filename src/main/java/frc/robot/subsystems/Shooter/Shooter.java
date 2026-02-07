// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.CANCoderCfg;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.Vision.PhotonCameraCfg;
import frc.robot.subsystems.Vision.PhotonVisionCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.CANCoderCfg;

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
                                                                      
  private final PhotonVisionCamera turretCamera;


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

  private double shooterSetSpeed = 0.0;
  private double turretSetAngle = Math.toRadians(180.0);
  private double hoodSetAngle = 0.0;
  private Translation2d targetTranslation = new Translation2d(0,0);

  private boolean isTurretTargetPresent = false;
  private boolean isHubFound = false;
  private int targetId = -1;

  private enum ShooterState{
    OFF,
    WAIT,
    READY,
    SHOOTING,
    RECOVERY;
  }

  private enum TurretState{
    INACTIVE,
    MOVE_TO_TARGET,
    ON_TARGET;
  }

  private ShooterState shooterState = ShooterState.WAIT;
   
  private TurretState turretState = TurretState.MOVE_TO_TARGET;

  private DriveSubsystem drive;

  public Shooter(DriveSubsystem drive) {
    this.drive = drive;
    
    turretCamera = new PhotonVisionCamera(PhotonCameraCfg.TURRET_CAM, PhotonCameraCfg.TURRET_CAM_TRANSFORM);
    
    configShooterMotors();
    configFeedMotor();
    configHoodMotor();
    configIndexMotor();
    configTurretMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTurretCamera();
    updateHoodAngleSetPoint();
    updateShooterSetPoint();
    updateTurretAngleSetPoint();
    updateShooterState();
    updateTurretState();
    updateDashboard();
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
    hoodCANcoderConfig.MagnetSensor.MagnetOffset = -ShooterCfg.HOOD_ABS_ENCODER_OFFSET;
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
    turretCANcoderConfig.MagnetSensor.MagnetOffset = -ShooterCfg.TURRET_ABS_ENCODER_OFFSET;
    turretCANcoderConfig.MagnetSensor.SensorDirection = ShooterCfg.TURRET_CAN_CODER_DIRECTION;
    turretCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ShooterCfg.DISCONTINUITY_POINT;
    turretAbsEncoder.getConfigurator().apply(turretCANcoderConfig);
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
    return angle.getValueAsDouble()*2.0*Math.PI;
  }

  public void setShooterSpeed(double speed){
    shooterSetSpeed = speed;
  }

  public void setTurretAngle(double angle){
    turretSetAngle = angle;
  }

  public void setHoodAngle(double angle){
    hoodSetAngle = angle;
  }

  private void updateTurretCamera(){
    var turretCamResults = turretCamera.getLatestPipelineTargets();
    var alliance = DriverStation.getAlliance();
    targetId = -1;

    isTurretTargetPresent = !turretCamResults.isEmpty();

    if(!turretCamResults.isEmpty()){
      if(alliance.isPresent()){
        for(int i = 0;i<turretCamResults.size();i++){
          var tagId = turretCamResults.get(i).getFiducialId();
          if(alliance.get() == DriverStation.Alliance.Blue){
            //Blue alliance so we only care if the tag is on the alliance zone side of the blue hub
            if((tagId == 18)||(tagId == 19)||(tagId == 20)||
               (tagId == 21)||(tagId == 24)||(tagId == 27)){
                  //Found ID is on the alliance zone side of the blue hub  
                  targetId = tagId;
                  isHubFound = true;
            }else{
              //Found ID is NOT on the alliance zone side of the blue hub
              targetId = -1;
              isHubFound = false;
            }
          }else if((tagId == 5)||(tagId == 8)||(tagId == 9)||
                   (tagId == 10)||(tagId == 11)||(tagId == 12)){
                    //Found ID is on the alliance zone side of the red hub
                    targetId = tagId;
                    isHubFound = true;
            }else{
              //Found ID is NOT on the alliance zone side of the red hub
              targetId = -1;
              isHubFound = false;
          }
        }
      }else{
        //Alliance is not present in DS data
        targetId = -1;
        isHubFound = false;
      }
    }else{
      //TurretResults is empty
      targetId = -1;
      isHubFound = false;
    }
  }

  private void updateDashboard(){
    SmartDashboard.putNumber("Turret Angle", getTurretAbsPositionZeroed());
    SmartDashboard.putNumber("Turret Velocity", getTurretAbsVelocity());
    SmartDashboard.putNumber("Turret Motor Command", turretMotor.getAppliedOutput());
    SmartDashboard.putString("Turret State", turretState.toString());
    SmartDashboard.putNumber("Turret Set Angle", turretSetAngle);
    SmartDashboard.putBoolean("Is Turret At Set Point", turretPIDController.atSetpoint());
    SmartDashboard.putNumber("Target Translation X", targetTranslation.getX());
    SmartDashboard.putNumber("Target Translation Y", targetTranslation.getY());
    SmartDashboard.putNumber("Angle to Target", calculateTargetAngle(targetTranslation));
    SmartDashboard.putNumber("Distance To Target", calculateTargetDistance(targetTranslation));
    SmartDashboard.putBoolean("Is Hub Found", isHubFound);
    SmartDashboard.putBoolean("Is Turret Target Found", isTurretTargetPresent);
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
        shooterSetSpeed = lookupShooterSpeed(targetTranslation);
        hoodSetAngle = lookupHoodAngle();
        if((shooterPIDController.isAtSetpoint())&&
           (hoodPIDController.atSetpoint())){
            shooterState = ShooterState.READY;
        }else{
          //DO NOTHING
        }
        break;
      case READY:
        shooterSetSpeed = lookupShooterSpeed(targetTranslation);
        hoodSetAngle = lookupHoodAngle();
        if((!shooterPIDController.isAtSetpoint())||
           (!hoodPIDController.atSetpoint())){
            shooterState = ShooterState.WAIT;
        }else{
          //DO NOTHING
        }
        break;
      case SHOOTING:
        shooterSetSpeed = lookupShooterSpeed(targetTranslation);
        hoodSetAngle = lookupHoodAngle();
        if((!shooterPIDController.isAtSetpoint())||
           (!hoodPIDController.atSetpoint())){
            shooterState = ShooterState.RECOVERY;
        }else{
          //DO NOTHING
        }
        //DO NOTHING
      break;
      case RECOVERY:
        shooterSetSpeed = lookupShooterSpeed(targetTranslation);
        hoodSetAngle = lookupHoodAngle();
        if((shooterPIDController.isAtSetpoint())&&
           (hoodPIDController.atSetpoint())){
            shooterState = ShooterState.SHOOTING;
        }else{
          //DO NOTHING
        }
      break;
    } 
  }

  private void setShooterToWait(){
    shooterState = ShooterState.WAIT;
    setIndexSpeed(0);
    setFeedSpeed(0);
    shooterSetSpeed = lookupShooterSpeed(targetTranslation);
    hoodSetAngle = lookupHoodAngle();
  }
  private void setShooterOn(){
    shooterState = ShooterState.SHOOTING;
    setIndexSpeed(ShooterCfg.INDEX_SPEED);
    setFeedSpeed(ShooterCfg.FEED_SPEED);
  }
  private void setShooterOff(){
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
        if(turretPIDController.atSetpoint()){
          turretState = TurretState.ON_TARGET;
        } else{
          //Do nothing
        } 
        break;
      case ON_TARGET:
        targetTranslation = calculateTargetPosition();
        turretSetAngle = calculateTargetAngle(targetTranslation);
        if(!turretPIDController.atSetpoint()){
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

  private double lookupShooterSpeed(Translation2d targetPose){
    //TODO Look UP shooter speed and set the shooterSetSpeed to the lookup value
    var distance = calculateTargetDistance(targetPose); 
    //return ShooterLookup.LOOKUP[(int)distance][0];
    return 0; //CL - Was causing array out of bounds need to debug
  }
  private double lookupHoodAngle(){
    //TODO Look UP Hood Angle and set the hoodAngle to the lookup value
    return 0;
  }

  private double calculateTargetAngle(Translation2d targetPose){
    var angle = drive.getDistanceAngleToPoint(targetPose).getY();
    if (angle<ShooterCfg.TURRET_MIN_ANGLE){
      angle = ShooterCfg.TURRET_MIN_ANGLE;
    }else if (angle>ShooterCfg.TURRET_MAX_ANGLE){
      angle = ShooterCfg.TURRET_MAX_ANGLE;
    }else{

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
        if(robotPose.getX() > 12.5){
          return ShooterCfg.RED_HUB_TARGET_POSE;
        }else if(robotPose.getY() >= 4.03){
          return ShooterCfg.RED_LEFT;
        }else{
          return ShooterCfg.RED_RIGHT;
        }
      }else{
        if(robotPose.getX() < 4.54){
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
}

