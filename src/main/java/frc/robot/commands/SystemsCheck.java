//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Driver Controller LEFT BUMPER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class SystemsCheck extends Command {
  /** Creates a new SystemsCheck. */
  private DriveSubsystem drive;
  private Intake intake;
  private Shooter shooter;

//Making alllllllll of the drive motors exist becuase swerve only configs them in the moduels & not as individually running motors :(
  public static final int BACK_LEFT_TURN_ID = 4;
  public static final int BACK_LEFT_DRIVE_ID = 5;

  public static final int BACK_RIGHT_TURN_ID = 8;
  public static final int BACK_RIGHT_DRIVE_ID = 9;
  
  public static final int FRONT_RIGHT_TURN_ID = 10;
  public static final int FRONT_RIGHT_DRIVE_ID = 11;

  public static final int FRONT_LEFT_TURN_ID = 16;
  public static final int FRONT_LEFT_DRIVE_ID = 17;

//Create the SparkFlex
  public static final SparkFlex BACK_LEFT_DRIVE = new SparkFlex(BACK_LEFT_DRIVE_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex BACK_LEFT_TURN = new SparkFlex(BACK_LEFT_TURN_ID, SparkLowLevel.MotorType.kBrushless);

  public static final SparkFlex BACK_RIGHT_DRIVE = new SparkFlex(BACK_RIGHT_DRIVE_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex BACK_RIGHT_TURN = new SparkFlex(BACK_RIGHT_TURN_ID, SparkLowLevel.MotorType.kBrushless);

  public static final SparkFlex FRONT_RIGHT_DRIVE = new SparkFlex(FRONT_RIGHT_DRIVE_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex FRONT_RIGHT_TURN = new SparkFlex(FRONT_RIGHT_TURN_ID, SparkLowLevel.MotorType.kBrushless);

  public static final SparkFlex FRONT_LEFT_DRIVE = new SparkFlex(FRONT_LEFT_DRIVE_ID, SparkLowLevel.MotorType.kBrushless);
  public static final SparkFlex FRONT_LEFT_TURN = new SparkFlex(FRONT_LEFT_TURN_ID, SparkLowLevel.MotorType.kBrushless);

//Make it usable...

  private final SparkFlex frontLeftDrive = FRONT_LEFT_DRIVE;
  private final SparkFlex frontLeftTurn = FRONT_LEFT_TURN;

  private final SparkFlex frontRightDrive = FRONT_RIGHT_DRIVE;
  private final SparkFlex frontRightTurn = FRONT_RIGHT_TURN;

  private final SparkFlex backLeftDrive = BACK_LEFT_DRIVE;
  private final SparkFlex backLeftTurn = BACK_LEFT_TURN;

  private final SparkFlex backRightDrive = BACK_RIGHT_DRIVE;
  private final SparkFlex backRightTurn = BACK_RIGHT_TURN;



  public SystemsCheck(DriveSubsystem drive, Intake intake, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(intake);
    addRequirements(shooter);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
  }

//Set the drive & turn speeds to use for test
  public void setDriveSpeed(double speed){
    frontLeftDrive.set(speed);
    frontRightDrive.set(speed);
    backLeftDrive.set(speed);
    backRightDrive.set(speed);
  }

  public void setTurnSpeed(double speed){
    frontLeftTurn.set(speed);
    frontRightTurn.set(speed);
    backLeftTurn.set(speed);
    backRightTurn.set(speed);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(0, 0, 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled. (When the left bumper on the driver Controller is pressed)
  @Override
  public void execute() {
    //drive.drive(0.5,0,1, true);
    //systemsCheckShooter controlls index, shooter, feed, and hood
    shooter.systemsCheckShooter();
    //systemsCheckIntake just turns on the intake (nothing else)
    //intake.systemsCheckIntake();
  
    setDriveSpeed(0.25);
    setTurnSpeed(0.25);
  }

  // Called once the command ends or is interrupted. (When the left button on the driver controller is released)
  @Override
  public void end(boolean interrupted) {
   //  drive.drive(0, 0, 0, true);
    shooter.setShooterOff();
    intake.setIntakeOff(); 
    shooter.setIndexSpeed(0);
    shooter.setFeedSpeed(0);
    shooter.setHoodAngle(20);
    setDriveSpeed(0);
    setTurnSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static Command startSystemCheck(DriveSubsystem driveSubsystem, Intake intake, Shooter shooter) {
   return Commands.sequence(
    intake.systemsCheckIntakeCommand(),
    shooter.systemsCheckShooterCommand()
   );
  }
}

