// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final Solenoid climberSolenoid1 = ClimberCfg.CLIMBER_SOLENOID1;
  private final Solenoid climberSolenoid2 = ClimberCfg.CLIMBER_SOLENOID2;
  private final AnalogInput climberPSI = ClimberCfg.CLIMBER_PSI;

  private boolean climberIn = true;
  private double climberPSIValue = 0;

  DriveSubsystem drive = new DriveSubsystem();

  public Climber(DriveSubsystem drive) {
    this.drive = drive;
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculatePressure();
    
    if(drive.isInTrenchZone()){
      setClimberIn();
    }
    
    updateDashboard();


  }
  public void setClimberOut(){
    climberIn = false;    
    climberSolenoid1.set(true);
    climberSolenoid2.set(true);
  }
  public void setClimberIn(){
    climberIn = true;
    climberSolenoid1.set(false);
    climberSolenoid2.set(false);
  }

  public void calculatePressure(){
    var climberVoltage = climberPSI.getVoltage();
    climberPSIValue = (climberVoltage - 0.5)*50;
  }

  public boolean isClimberReady(){
    return(climberPSIValue>= ClimberCfg.CLIMBER_PSI_LIMIT);
  }

  private void updateDashboard(){
    SmartDashboard.putNumber("Climber Pressure", climberPSIValue);
    SmartDashboard.putBoolean("Climber Ready", isClimberReady());
  }
  
  public void toggleClimber(){
    if(climberIn == true){
      setClimberOut();
    }else{
      setClimberIn();
    }
  }
}
