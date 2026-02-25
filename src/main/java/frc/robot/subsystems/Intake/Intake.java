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

  public Intake() {
    intakeMotorConfig.inverted(IntakeCfg.INTAKE_MOTOR_REVERSED);
    intakeMotorConfig.idleMode(IntakeCfg.INTAKE_MOTOR_IDLE_MODE);
    intakeMotorConfig.smartCurrentLimit(IntakeCfg.INTAKE_MOTOR_CURRENT_LIMIT);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeOn(){
    intakeMotor.set(1.0);
  }

  public void setIntakeReverse(){
    intakeMotor.set(-1.0);
  }

  public void setIntakeOff(){
    intakeMotor.set(0);
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
  }
}
