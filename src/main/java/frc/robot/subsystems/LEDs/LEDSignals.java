// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSignals extends SubsystemBase {

  /** Creates a new LEDSignals. */
  private final Spark signalLED = LEDSignalCfg.SIGNAL_LED;

  public LEDSignals() {}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLEDOutput(double output){
    signalLED.set(output);
  }
}
