package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class SystemsCheck2 {
  public static Command startSystemCheck(DriveSubsystem driveSubsystem, Intake intake, Shooter shooter) {
    return Commands.sequence(
        Commands.print("SYSTEM CHECK START"),
        intake.systemsCheckIntakeCommand(),
        new WaitCommand(3),
        shooter.systemsCheckShooterCommand(),
        Commands.print("SYSTEM CHECK COMPLETE")
    );
  }

}
