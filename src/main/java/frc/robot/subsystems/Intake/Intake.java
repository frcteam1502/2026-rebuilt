// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final SparkFlex intakeMotor = IntakeCfg.INTAKE_MOTOR;
  private final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
  private final Solenoid hopperSolenoid = IntakeCfg.INTAKE_SOLENOID;

  private boolean hopperIn = true;
  private boolean isShooterRequestIn = false;
  private boolean isShooterRequestOut = false;
  private boolean isCntrlRequestIn = false;
  private boolean isCntrlRequestOut = false;
  private boolean isShooterRequestSlow = false;
  
    public Intake() {
      intakeMotorConfig.inverted(IntakeCfg.INTAKE_MOTOR_REVERSED);
      intakeMotorConfig.idleMode(IntakeCfg.INTAKE_MOTOR_IDLE_MODE);
      intakeMotorConfig.smartCurrentLimit(IntakeCfg.INTAKE_MOTOR_CURRENT_LIMIT);
  
      intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //arbitrateIntake();
    }

    private void arbitrateIntake(){
      if(isCntrlRequestIn){
        intakeMotor.set(1.0);
      }else if(isCntrlRequestOut){
        intakeMotor.set(-1.0);
      }else if(isShooterRequestIn){
        intakeMotor.set(1.0);
      }else if(isShooterRequestSlow){
        intakeMotor.set(0.5);
      }else if(isShooterRequestOut){
        intakeMotor.set(-1.0);
      }else{
        intakeMotor.set(0);
      }
    }
  
    public void setIntakeOn(){
      isCntrlRequestIn = true;
      isCntrlRequestOut = false;
    }
  /* 
    public void setIntakeReverse(){
      isCntrlRequestIn = false;
      isCntrlRequestOut = true;
    }
  
    public void setIntakeOff(){
      isCntrlRequestIn = false;
      isCntrlRequestOut = false;
    }
  
    public void setHopperOut(){
      hopperSolenoid.set(true);
    }
  
     public void setHopperIn(){
      hopperSolenoid.set(false);
    }
    public void setIntakeSpeed(double speed){
      intakeMotor.set(speed);
    }
  
  public void shooterRequestIntakeOn(){
    isShooterRequestIn = true;
    isShooterRequestSlow = false;
    isShooterRequestOut = false;
  }

  public void shooterRequestIntakeOnSlow(){
    isShooterRequestIn = false;
    isShooterRequestSlow = true;
    isShooterRequestOut = false;
  }

  public void shooterRequestIntakeReverse(){
    isShooterRequestIn = false;
    isShooterRequestSlow = false;
    isShooterRequestOut = true;
  }

  public void shooterRequestIntakeOff(){
    isShooterRequestIn = false;
    isShooterRequestSlow = false;
    isShooterRequestOut = false;
  }


  public void toggleHopper(){
    if(hopperIn == false){
      setHopperIn();
      hopperIn = true;
    }else{
      setHopperOut();
      hopperIn = false;
    }
  }
  public boolean isHopperIn(){
    return hopperIn;
  }*/
}
