// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDSignals;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final SparkFlex leadIntakeMotor = IntakeCfg.LEAD_INTAKE_MOTOR;
  private final SparkFlex followerIntakeMotor = IntakeCfg.FOLLOWER_INTAKE_MOTOR;
  private final SparkFlexConfig leadIntakeMotorConfig = new SparkFlexConfig();
  private final SparkFlexConfig followerIntakeMotorConfig = new SparkFlexConfig();
  private final Solenoid hopperSolenoid = IntakeCfg.INTAKE_SOLENOID;

  private boolean hopperIn = true;
  private boolean isShooterRequestIn = false;
  private boolean isShooterRequestOut = false;
  private boolean isCntrlRequestIn = false;
  private boolean isCntrlRequestOut = false;
  private boolean isShooterRequestSlow = false;

  private Timer intakeTimer;
  
    public Intake() {
      leadIntakeMotorConfig.inverted(IntakeCfg.LEAD_INTAKE_MOTOR_REVERSED);
      leadIntakeMotorConfig.idleMode(IntakeCfg.INTAKE_MOTOR_IDLE_MODE);
      leadIntakeMotorConfig.smartCurrentLimit(IntakeCfg.INTAKE_MOTOR_CURRENT_LIMIT);

      followerIntakeMotorConfig.follow(IntakeCfg.LEAD_INTAKE_MOTOR_ID, true);
      followerIntakeMotorConfig.idleMode(IntakeCfg.INTAKE_MOTOR_IDLE_MODE);
      followerIntakeMotorConfig.smartCurrentLimit(IntakeCfg.INTAKE_MOTOR_CURRENT_LIMIT);
  

      leadIntakeMotor.configure(leadIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      followerIntakeMotor.configure(followerIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      arbitrateIntake();
      SmartDashboard.putData(LEDSignals.signalLED);
    }

    private void arbitrateIntake(){
      if(isCntrlRequestIn){
        leadIntakeMotor.set(1.0);
        LEDSignals.hopperInColor();
      }else if(isCntrlRequestOut){
        leadIntakeMotor.set(-1.0);
        LEDSignals.hopperOutColor();
      }else if(isShooterRequestIn){
        leadIntakeMotor.set(1.0);
        LEDSignals.intakeOnInColor();
      }else if(isShooterRequestSlow){
        leadIntakeMotor.set(0.5);
        LEDSignals.intakeOnInColor();
      }else if(isShooterRequestOut){
        leadIntakeMotor.set(-1.0);
        LEDSignals.intakeOnOutColor();
      }else{
        leadIntakeMotor.set(0);
      }
    }
  
    public void setIntakeOn(){
      isCntrlRequestIn = true;
      isCntrlRequestOut = false;
    }
  
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
      hopperIn = false;
      isCntrlRequestIn = true;
      isCntrlRequestOut = false;
    }
     public void setHopperIn(){
      hopperIn = true;
      hopperSolenoid.set(false);
    }
    public void setIntakeSpeed(double speed){
      leadIntakeMotor.set(speed);
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
    }else{
      setHopperOut();
    }
  }
  public boolean isHopperIn(){
    return hopperIn;
    
  }
}
