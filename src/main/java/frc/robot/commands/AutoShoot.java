// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  private Shooter shooter;
  private Timer fuelTimer;
  
    /** Creates a new AutoShoot. */
    public AutoShoot() {
      // Use addRequirements() here to declare subsystem dependencies.
      this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Set shooter to on (Won't run untill setShooterWait is complete)
    shooter.setShooterOn();
    //Starts Fuel Timer to check if we're done shooting
    fuelTimer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get Values from lazer can If(lazserCan = true) reset timer else, null
   /* 
   if(Shooter.lazserCanValue == true){
        fuelTimer.reset();
    }*/ 
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterToWait();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if fuel timer > x return true else return false
    if (fuelTimer.get() > 3) {
      return true;
    } else {
      return false;
    }
  }
}
