// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class SystemsCheck extends Command {
  /** Creates a new SystemsCheck. */
 /* private DriveSubsystem drive;
  private Intake intake;
  private Shooter shooter;


  public SystemsCheck(DriveSubsystem drive, Intake intake, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(intake);
    addRequirements(shooter);
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(0, 0, 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(0.5,0,1, true);
    shooter.systemsCheckShooter();
    intake.systemsCheckIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     drive.drive(0, 0, 0, true);
     shooter.setShooterOff();
     intake.setIntakeOff(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}*/
