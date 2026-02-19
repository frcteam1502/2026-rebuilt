// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final Solenoid climberSolenoid1 = ClimberCfg.CLIMBER_SOLENOID1;
  private final Solenoid climberSolenoid2 = ClimberCfg.CLIMBER_SOLENOID2;

  private boolean climberIn = true;


  public Climber() {}
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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
  
  public void toggleClimber(){
    if(climberIn == true){
      setClimberOut();
    }else{
      setClimberIn();
    }
  }
}
