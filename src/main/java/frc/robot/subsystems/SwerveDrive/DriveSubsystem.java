package frc.robot.subsystems.SwerveDrive;

import frc.robot.Driver;
import frc.robot.Logger;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterCfg;
import frc.robot.subsystems.Shooter.ShooterLookup;
import frc.robot.subsystems.Vision.PhotonCameraCfg;
import frc.robot.subsystems.Vision.PhotonVisionCamera;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase{

  private final Field2d m_field = new Field2d(); 
   StructPublisher<Pose2d> robotPublisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("robotPose", Pose2d.struct)
        .publish();
  StructPublisher<Pose2d> targetPublisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("targetPose", Pose2d.struct)
        .publish();
  StructPublisher<Pose2d> aimPublisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("aimPose", Pose2d.struct)
        .publish();
 
  public static boolean isTeleOp = false;

  public boolean isTurning = false;
  public double turnCommand = 0.0;
  public double fieldXCommand = 0;
  public double fieldYCommand = 0;
  public double fieldRotCommand = 0;

  ChassisSpeeds speedCommands = new ChassisSpeeds(0, 0, 0);
  ChassisSpeeds relativeCommands = new ChassisSpeeds(0,0,0);

  private final SwerveModule frontLeft = new SwerveModule(
    DrivebaseCfg.FRONT_LEFT_MOD_ID,
    ChassisMotorCfg.DRIVE_FRONT_LEFT, ChassisMotorCfg.ANGLE_FRONT_LEFT, 
    CANCoderCfg.FRONT_LEFT_CAN_CODER);

  private final SwerveModule frontRight = new SwerveModule(
    DrivebaseCfg.FRONT_RIGHT_MOD_ID,
    ChassisMotorCfg.DRIVE_FRONT_RIGHT, ChassisMotorCfg.ANGLE_FRONT_RIGHT, 
    CANCoderCfg.FRONT_RIGHT_CAN_CODER);

  private final SwerveModule backLeft = new SwerveModule(
    DrivebaseCfg.BACK_LEFT_MOD_ID,
    ChassisMotorCfg.DRIVE_BACK_LEFT, ChassisMotorCfg.ANGLE_BACK_LEFT, 
    CANCoderCfg.BACK_LEFT_CAN_CODER);

  private final SwerveModule backRight = new SwerveModule(
    DrivebaseCfg.BACK_RIGHT_MOD_ID,
    ChassisMotorCfg.DRIVE_BACK_RIGHT, ChassisMotorCfg.ANGLE_BACK_RIGHT, 
    CANCoderCfg.BACK_RIGHT_CAN_CODER);

  private final Pigeon2 gyro = IMU_Cfg.IMU;

  private final SwerveDriveKinematics kinematics = DrivebaseCfg.KINEMATICS;

  //public final SwerveDrivePoseEstimator odometry;
  public final SwerveDriveOdometry odometry;

  public final SwerveDrivePoseEstimator poseEstimator;

  private final PhotonVisionCamera leftPhotonCamera;
  private final PhotonVisionCamera rightPhotonCamera;

  private Pose2d pose = new Pose2d();
  private Pose2d photonLeftPose = new Pose2d();
  private Pose2d photonRightPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  SwerveModuleState[] loggerSwerveCommands;

  //Debug
  private double angleRadians = 0;

  //Create a SysIdRoutine object for characterizing the drive
  private final SysIdRoutine sysIdLinear = 
  new SysIdRoutine(
    //Create a new SysID Congig with default ramp rate (0.1 V/s), step (7V), and time out values
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      voltage -> {
        frontLeft.setSysIDVoltage(voltage, 0);
        frontRight.setSysIDVoltage(voltage, 0);
        backLeft.setSysIDVoltage(voltage, 0);
        backRight.setSysIDVoltage(voltage, 0);},
      // Tell SysId how to record a frame of data for each motor on the mechanism being
      // characterized.
      log -> {
        //Log a frame for the frontLeft Motor
        log.motor("drive-frontLeft")
          .voltage(frontLeft.getDriveMotorVoltage())
          .linearPosition(frontLeft.getLinearPositionMeters())
          .linearVelocity(frontLeft.getModuleVelocityMetersPerSec());
        //Log a frame for the frontRight Motor
        log.motor("drive-frontRight")
          .voltage(frontRight.getDriveMotorVoltage())
          .linearPosition(frontRight.getLinearPositionMeters())
          .linearVelocity(frontRight.getModuleVelocityMetersPerSec());
        //Log a frame for the backLeft Motor
        log.motor("drive-backLeft")
          .voltage(backLeft.getDriveMotorVoltage())
          .linearPosition(backLeft.getLinearPositionMeters())
          .linearVelocity(backLeft.getModuleVelocityMetersPerSec());
        //Log a frame for the backRight Motor
        log.motor("drive-backRight")
          .voltage(backRight.getDriveMotorVoltage())
          .linearPosition(backRight.getLinearPositionMeters())
          .linearVelocity(backRight.getModuleVelocityMetersPerSec());
      },
      // Tell SysId to make generated commands require this subsystem, suffix test state in
      // WPILog with this subsystem's name ("drive")
      this));
  //Create a SysIdRoutine object for characterizing the drive
  private final SysIdRoutine sysIdAngular = 
  new SysIdRoutine(
    //Create a new SysID Congig with default ramp rate (0.1 V/s), step (7V), and time out values
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      voltage -> {
        frontLeft.setSysIDVoltage((voltage), -(Math.PI/4));
        frontRight.setSysIDVoltage(voltage, (Math.PI/4));
        backLeft.setSysIDVoltage(voltage, (Math.PI/4));
        backRight.setSysIDVoltage(voltage, -(Math.PI/4));},
      // Tell SysId how to record a frame of data for each motor on the mechanism being
      // characterized.
      log -> {
        //Log a frame for the frontLeft Motor
        log.motor("drive-frontLeft")
          .voltage(frontLeft.getDriveMotorVoltage())
          .linearPosition(frontLeft.getLinearPositionMeters())
          .linearVelocity(frontLeft.getModuleVelocityMetersPerSec());
        //Log a frame for the frontRight Motor
        log.motor("drive-frontRight")
          .voltage(frontRight.getDriveMotorVoltage())
          .linearPosition(frontRight.getLinearPositionMeters())
          .linearVelocity(frontRight.getModuleVelocityMetersPerSec());
        //Log a frame for the backLeft Motor
        log.motor("drive-backLeft")
          .voltage(backLeft.getDriveMotorVoltage())
          .linearPosition(backLeft.getLinearPositionMeters())
          .linearVelocity(backLeft.getModuleVelocityMetersPerSec());
        //Log a frame for the backRight Motor
        log.motor("drive-backRight")
          .voltage(backRight.getDriveMotorVoltage())
          .linearPosition(backRight.getLinearPositionMeters())
          .linearVelocity(backRight.getModuleVelocityMetersPerSec());
      },
      // Tell SysId to make generated commands require this subsystem, suffix test state in
      // WPILog with this subsystem's name ("drive")
      this));

  public Command sysIdLinearQuasistatic(SysIdRoutine.Direction direction){
    return sysIdLinear.quasistatic(direction);
  }

  public Command sysIdLinearDynamic(SysIdRoutine.Direction direction){
    return sysIdLinear.dynamic(direction);
  }

  public Command sysIdAngularQuasistatic(SysIdRoutine.Direction direction){
    return sysIdAngular.quasistatic(direction);
  }

  public Command sysIdAngularDynamic(SysIdRoutine.Direction direction){
    return sysIdAngular.dynamic(direction);
  }

  public DriveSubsystem() {
    resetGyro(0);
    this.odometry = new SwerveDriveOdometry(kinematics, getGyroRotation2d(), getModulePositions());

    this.poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      getGyroRotation2d(), 
      getModulePositions(),
      estimatedPose,
      createStateStdDevs(
          PoseEstCfg.POSITION_STD_DEV_X,
          PoseEstCfg.POSITION_STD_DEV_Y,
          PoseEstCfg.POSITION_STD_DEV_THETA),
      createVisionMeasurementStdDevs(
          PoseEstCfg.VISION_STD_DEV_X,
          PoseEstCfg.VISION_STD_DEV_Y,
          PoseEstCfg.VISION_STD_DEV_THETA));
        
    robotAimPIDController= new PIDController(DrivebaseCfg.ROBOT_AIM_P_GAIN,
                                            DrivebaseCfg.ROBOT_AIM_I_GAIN,
                                            DrivebaseCfg.ROBOT_AIM_D_GAIN);

    robotAimPIDController.enableContinuousInput(-Math.PI, Math.PI);


    
    leftPhotonCamera = new PhotonVisionCamera(PhotonCameraCfg.LEFT_APRILTAG_CAM, 
          PhotonCameraCfg.LEFT_APRILTAG_CAM_TRANSFORM);

    rightPhotonCamera = new PhotonVisionCamera(PhotonCameraCfg.RIGHT_APRILTAG_CAM, 
          PhotonCameraCfg.RIGHT_APRILTAG_CAM_TRANSFORM);

    reset();
    registerLoggerObjects();

    //Add a Field2d widget to the Dashboard
    SmartDashboard.putData("Field", m_field);

    //Add a Swerve widget to the Dashboard
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle",     () -> frontLeft.getAbsPositionZeroed(), null);
        builder.addDoubleProperty("Front Left Velocity",  () -> frontLeft.getVelocity(), null);

        builder.addDoubleProperty("Front Right Angle",    () -> frontRight.getAbsPositionZeroed(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getVelocity(), null);

        builder.addDoubleProperty("Back Left Angle",      () -> backLeft.getAbsPositionZeroed(), null);
        builder.addDoubleProperty("Back Left Velocity",   () -> backLeft.getVelocity(), null);

        builder.addDoubleProperty("Back Right Angle",     () -> backRight.getAbsPositionZeroed(), null);
        builder.addDoubleProperty("Back Right Velocity",  () -> backRight.getVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> estimatedPose.getRotation().getRadians(), null);
      }
    });

    //Configure Auto Builder last!
    configAutoBuilder(); 
  }
  
  @Override
  public void periodic() {
    updateOdometry();
    updateEstimatedPose();
    updatePhotonVisionPose();

    //Update SmartDashboard 
    updateDashboard();
  }
  boolean toggleAim;
  boolean m_atSetPoint;
  public void setAutoTargetOn(){
    toggleAim = true;
  }
  public void setAutoTargetOff(){
    toggleAim = false;
    m_atSetPoint = false;
  }
  public boolean atSetPoint(){
    return m_atSetPoint;
  }
  public void setSwerveXLock(){
    SwerveModuleState[] SwerveLockStates = new SwerveModuleState[]{
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
    };
    setDesiredModuleStates(SwerveLockStates);
  }
  private double getRotation(double rot){
    if (toggleAim){
      //rotation of robot to target
      var targetPose = Shooter.calculateTargetPosition(this);
      return getOmega(targetPose);
    }
    return rot;
  }
  //Drive command consumer
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //Set Dashboard variables
    fieldXCommand = xSpeed;
    fieldYCommand = ySpeed;
    fieldRotCommand = getRotation(rot);
    
    if(fieldRelative){
      var alliance = DriverStation.getAlliance();
      if((alliance.isPresent()) && (alliance.get() == DriverStation.Alliance.Red)){
        speedCommands = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, fieldRotCommand, getGyroRotation2d());
      }else{
        speedCommands = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, fieldRotCommand, getGyroRotation2d());   
      }  
    } else {
      speedCommands.omegaRadiansPerSecond = fieldRotCommand;
      speedCommands.vxMetersPerSecond = xSpeed;
      speedCommands.vyMetersPerSecond = ySpeed;
    }

    driveRobotRelative(speedCommands);
  }

  //ChassisSpeed consumer
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    //This method is a consumer of ChassisSpeed and sets the corresponding module states.  This is required for PathPlanner 2024
    //Save off to SmartDashboard
    relativeCommands.vxMetersPerSecond = robotRelativeSpeeds.vxMetersPerSecond;
    relativeCommands.vyMetersPerSecond = robotRelativeSpeeds.vyMetersPerSecond;
    relativeCommands.omegaRadiansPerSecond = robotRelativeSpeeds.omegaRadiansPerSecond;

    ChassisSpeeds discreteChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
    //Convert from robot frame of reference (ChassisSpeeds) to swerve module frame of reference (SwerveModuleState)
    var swerveModuleStates = kinematics.toSwerveModuleStates(discreteChassisSpeeds);

    //Normalize wheel speed commands to make sure no speed is greater than the maximum achievable wheel speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseCfg.MAX_SPEED_METERS_PER_SECOND);

    //Save the requested states for the logger
    loggerSwerveCommands = swerveModuleStates;

    //Set the speed and angle of each module
    if (m_isXLocked){
      setDesiredModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
      });
    }else{
      setDesiredModuleStates(swerveModuleStates);
    }
  }
  
  boolean m_isXLocked=false;
  public void setLock(boolean isLocked){
    m_isXLocked = isLocked;
  }
  //Interface with swerve modules
  private void setDesiredModuleStates(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  private void updateOdometry() {
    pose = odometry.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  private void updateEstimatedPose(){
    estimatedPose = poseEstimator.update(getGyroRotation2d(), getModulePositions());
  }

  private void updatePhotonVisionPose(){
    var leftPoseEstimate = leftPhotonCamera.processCamera(getEstimatedPose2d());

    if(leftPoseEstimate.isPresent()){
      photonLeftPose = leftPoseEstimate.get().estimatedPose.toPose2d();
      var timestampLeft = leftPoseEstimate.get().timestampSeconds;

      poseEstimator.addVisionMeasurement(photonLeftPose,
                                         timestampLeft,
                                         VecBuilder.fill(10,10,9999999));

    }

    var rightPoseEstimate = rightPhotonCamera.processCamera(getEstimatedPose2d());

    if(rightPoseEstimate.isPresent()){
      photonRightPose = rightPoseEstimate.get().estimatedPose.toPose2d();
      var timestampRight = rightPoseEstimate.get().timestampSeconds;
      poseEstimator.addVisionMeasurement(photonRightPose,
                                         timestampRight,
                                         VecBuilder.fill(10,10,9999999));
    }
  }

  private void updateDashboard(){

    //Field Oriented inputs
    SmartDashboard.putNumber("Field Oriented X Command (Forward)", fieldXCommand);
    SmartDashboard.putNumber("Field Oriented Y Command (Forward)", fieldYCommand);
    SmartDashboard.putNumber("Drive Robot Relative Rotation Command", relativeCommands.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Gyro Yaw", getIMU_Yaw());
    

    //Pose Info
    m_field.setRobotPose(estimatedPose);
    var robotPose = getEstimatedPose2d();
    var targetPose = new Pose2d(Shooter.calculateTargetPosition(this), Rotation2d.kZero);
    robotPublisher.set(robotPose);
    targetPublisher.set(targetPose);
    var aimPose = targetPose.relativeTo(robotPose);
    var distance = aimPose.getTranslation().getNorm();
    var aimPosition = (new Translation2d(distance,0)).rotateBy(robotPose.getRotation().plus(Rotation2d.k180deg));
    var aimField = robotPose.getTranslation().plus(aimPosition);
    aimPublisher.set(new Pose2d(aimField, Rotation2d.kZero));

    //SmartDashboard.putData("EstimatedPose", estimatedPose);
    SmartDashboard.putNumber("EstimatedPose X", estimatedPose.getX());
    SmartDashboard.putNumber("EstimatedPose Y", estimatedPose.getY());
    SmartDashboard.putNumber("EstimatedPose Rotation", estimatedPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Auto Aim Set Angle", Math.toDegrees(getDistanceAngleToPoint(Shooter.calculateTargetPosition(this)).getY()));
    SmartDashboard.putNumber("Target Position Y",Shooter.calculateTargetPosition(this).getY());
    SmartDashboard.putNumber("Target Position X",Shooter.calculateTargetPosition(this).getX());
    SmartDashboard.putData("Aim PID", robotAimPIDController);
    SmartDashboard.putNumber("Aim P", robotAimPIDController.getP());
    SmartDashboard.putNumber("Aim I", robotAimPIDController.getI());
    SmartDashboard.putNumber("Aim D", robotAimPIDController.getD());
    SmartDashboard.putNumber("Target Aim PID", getOmega(Shooter.calculateTargetPosition(this)));
    SmartDashboard.putBoolean("Is Aiming", toggleAim);
    

    //Photonvision Stuff for Debugging - Comment out when not in use to save bandwidth
    /*SmartDashboard.putNumber("PhotonLeft Pose X", photonLeftPose.getX());
    SmartDashboard.putNumber("PhotonLeft Pose Y", photonLeftPose.getY());
    SmartDashboard.putNumber("PhotonLeft Pose Rotation", photonLeftPose.getRotation().getDegrees());

    SmartDashboard.putNumber("PhotonRight Pose X", photonRightPose.getX());
    SmartDashboard.putNumber("PhotonRight Pose Y", photonRightPose.getY());
    SmartDashboard.putNumber("PhotonRight Pose Rotation", photonRightPose.getRotation().getDegrees());

    SmartDashboard.putBoolean("Left Any Found", leftPhotonCamera.doesCameraHaveAnyTargets());
    SmartDashboard.putBoolean("Left Any New", leftPhotonCamera.doesCameraHaveNewTargets());
    SmartDashboard.putBoolean("Left Any Valid", leftPhotonCamera.doesCameraHaveAnyValidTargets());
    SmartDashboard.putBoolean("Right Any Found", rightPhotonCamera.doesCameraHaveAnyTargets());
    SmartDashboard.putBoolean("Right Any New", rightPhotonCamera.doesCameraHaveNewTargets());
    SmartDashboard.putBoolean("Right Any Valid", rightPhotonCamera.doesCameraHaveAnyValidTargets());*/
  }

    private void registerLoggerObjects(){
      Logger.RegisterSparkFlex("FL Drive", ChassisMotorCfg.DRIVE_FRONT_LEFT);
      Logger.RegisterSparkFlex("FR Drive", ChassisMotorCfg.DRIVE_FRONT_RIGHT);
      Logger.RegisterSparkFlex("RL Drive", ChassisMotorCfg.DRIVE_BACK_LEFT);
      Logger.RegisterSparkFlex("RR Drive", ChassisMotorCfg.DRIVE_BACK_RIGHT);

      Logger.RegisterSparkFlex("FL Turn", ChassisMotorCfg.ANGLE_FRONT_LEFT);
      Logger.RegisterSparkFlex("FR Turn", ChassisMotorCfg.ANGLE_FRONT_RIGHT);
      Logger.RegisterSparkFlex("RL Turn", ChassisMotorCfg.ANGLE_BACK_LEFT);
      Logger.RegisterSparkFlex("RR Turn", ChassisMotorCfg.ANGLE_BACK_RIGHT);

      Logger.RegisterPigeon(IMU_Cfg.IMU);

      Logger.RegisterCanCoder("FL Abs Position", CANCoderCfg.FRONT_LEFT_CAN_CODER);
      Logger.RegisterCanCoder("FR Abs Position", CANCoderCfg.FRONT_RIGHT_CAN_CODER);
      Logger.RegisterCanCoder("RL Abs Position", CANCoderCfg.BACK_LEFT_CAN_CODER);
      Logger.RegisterCanCoder("RR Abs Position", CANCoderCfg.BACK_RIGHT_CAN_CODER);


      Logger.RegisterDoubleSensor("Front Left Angle Command",   ()->frontLeft.getCommandedAngle());
      Logger.RegisterDoubleSensor("Front Right Angle Command",  ()->frontRight.getCommandedAngle());
      Logger.RegisterDoubleSensor("Back Left Angle Command",    ()->backLeft.getCommandedAngle());
      Logger.RegisterDoubleSensor("Back Right Angle Command",   ()->backRight.getCommandedAngle());

      Logger.RegisterDoubleSensor("Front Left Angle (radians)",  ()->frontLeft.getAbsPositionZeroed());
      Logger.RegisterDoubleSensor("Front Right Angle (radians)", ()->frontRight.getAbsPositionZeroed());
      Logger.RegisterDoubleSensor("Back Left Angle (radians)",   ()->backLeft.getAbsPositionZeroed());
      Logger.RegisterDoubleSensor("Back Right Angle (radians)",  ()->backRight.getAbsPositionZeroed());

      Logger.RegisterDoubleSensor("FL Drive Speed Command", ()->frontLeft.getCommandedSpeed());
      Logger.RegisterDoubleSensor("FR Drive Speed Command", ()->frontRight.getCommandedSpeed());
      Logger.RegisterDoubleSensor("RL Drive Speed Command", ()->backLeft.getCommandedSpeed());
      Logger.RegisterDoubleSensor("RR Drive Speed Command", ()->backRight.getCommandedSpeed());

      Logger.RegisterDoubleSensor("FL Drive Speed", ()->frontLeft.getVelocity());
      Logger.RegisterDoubleSensor("FR Drive Speed", ()->frontRight.getVelocity());
      Logger.RegisterDoubleSensor("RL Drive Speed", ()->backLeft.getVelocity());
      Logger.RegisterDoubleSensor("RR Drive Speed", ()->backRight.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    poseEstimator.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometryToEstimatedPose(){
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), poseEstimator.getEstimatedPosition());
  }

  public void resetPoseEstimation(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  
  private SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()};
  }

  private double getIMU_Yaw() {
    var currentHeading = gyro.getYaw(); 
    return(currentHeading.getValueAsDouble());
  }

  private double getIMU_YawRate(){
    var currentRate = gyro.getAngularVelocityZWorld();
    return(currentRate.getValueAsDouble());
  }

  //ChassisSpeed Supplier
  public ChassisSpeeds getRobotRelativeSpeeds(){
    //This method is a supplier of ChassisSpeeds as determined by the module states.  This is required for PathPlanner 2024
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  
  private SwerveModulePosition[] getModulePositions() {
    //Returns 
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  private Rotation2d getGyroRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getIMU_Yaw()));
  }

  public Pose2d getOdometryPose2d() {
    return odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose2d(){
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Calculates distance and angle (Rotation2d) from current pose to targetPoint
   * @param targetPoint
   * @return the 2D Translation of distance and Rotation2d
   */
  public Translation2d getDistanceAngleToPoint(Translation2d targetPoint){
    //Returns Translation2d with distance and angle to target point
    Translation2d currentPosition = new Translation2d(getEstimatedPose2d().getX(), getEstimatedPose2d().getY());

    double x1 = currentPosition.getX();
    double y1 = currentPosition.getY();
    double x2 = targetPoint.getX();
    double y2 = targetPoint.getY();

    double delta_y = y2 - y1;
    double delta_x = x2 - x1;

    angleRadians = Math.atan2(delta_y, delta_x);
    
    angleRadians = angleRadians - estimatedPose.getRotation().getRadians();

    if(angleRadians<0){
      angleRadians = (Math.PI*2) + angleRadians;
    }
  
    return new Translation2d(currentPosition.getDistance(targetPoint), angleRadians);
  }

  public double getOmega(Translation2d targetPoint){
    //Returns Translation2d with distance and angle to target point
    Translation2d currentPosition = new Translation2d(getEstimatedPose2d().getX(), getEstimatedPose2d().getY());

    double x1 = currentPosition.getX();
    double y1 = currentPosition.getY();
    double x2 = targetPoint.getX();
    double y2 = targetPoint.getY();

    double delta_y = y2 - y1;
    double delta_x = x2 - x1;

    angleRadians = Math.atan2(delta_y, delta_x)+Math.PI;

    //Tried to fix spinning - didn't work 3/16/26
    /*if(angleRadians<0){
      angleRadians = (Math.PI*2) + angleRadians;
    }*/

    var aimCommand = robotAimPIDController.calculate(estimatedPose.getRotation().getRadians(), angleRadians);

    return aimCommand;
  }
  
  private final PIDController robotAimPIDController;

  public void resetGyro(double angle) {
    gyro.setYaw(angle);
  }

  private void resetModules() {
    frontLeft.zeroModule();

    frontRight.zeroModule();
    backLeft.zeroModule();
    backRight.zeroModule();
  }

  private void reset() {
    resetModules();
    resetOdometry(pose);
  }

  public void resetGyroToPose(){
    //This method will get called from teleopInit() via RobotContainer
    //First, reset the gyro with the heading from the robot pose
    resetGyro(pose.getRotation().getDegrees());
    //Now, reset the pose updated gyro heading
    odometry.resetRotation(getGyroRotation2d());
    poseEstimator.resetRotation(getGyroRotation2d());
  }
  
  /**
   * Creates a vector of standard deviations for the states. Standard deviations of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Creates a vector of standard deviations for the vision measurements. Standard deviations of
   * global measurements from vision. Increase these numbers to trust global measurements from
   * vision less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  private void configAutoBuilder(){
    //Wrapper for AutoBuilder.configure, must be called from DriveTrain config....

    /*From Path Planner example code 
    https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java*/

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        //this::getOdometryPose2d, //Robot pose supplier
        this::getEstimatedPose2d,
        this::resetOdometry, //Method to reset odometry (will be called if the robot has a starting pose)
        this::getRobotRelativeSpeeds, //ChassisSpeeds provider.  MUST BE ROBOT RELATIVE!!! 
        this::driveRobotRelative, //ChassisSpeeds consumer.  MUST BE ROBOT RELATIVE!!!
        new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0), //Translation PID constants
                new PIDConstants(6, 0, 0)), //Rotation PID constants
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()){
                return alliance.get() == DriverStation.Alliance.Red;
            }
              return false;
          },
        this //Reference to this subsystem to set 
      );

    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  //Rebuilt stuff
  public boolean isInTrenchZone(){
    var pose_x = getEstimatedPose2d().getX();
    
    if((pose_x > ShooterCfg.LOW_RED_TRENCHES)  &&
       (pose_x < ShooterCfg.HIGH_RED_TRENCHES))
    {
        return true;
    }else if((pose_x > ShooterCfg.LOW_BLUE_TRENCHES)&&
             (pose_x < ShooterCfg.HIGH_BLUE_TRENCHES)){
        return true;
    }else{
        return false;
    }
  }
}
