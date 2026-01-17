// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final SparkFlex intakeMotor = new SparkFlex(3, SparkLowLevel.MotorType.kBrushless);
  
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeIn(){
    intakeMotor.set(1.0);
  }

  public void setIntakeOut(){
    intakeMotor.set(-1.0);
  }

  public void setIntakeOff(){
    intakeMotor.set(0);
  }
}
