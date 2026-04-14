// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSignals extends SubsystemBase {
    public static final Spark signalLED = LEDSignalCfg.SIGNAL_LED;

  public LEDSignals() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void hopperInColor(){
    signalLED.set(0.67); //Gold
  }

  public static void hopperOutColor(){
    signalLED.set(0.89); //Blue violet
  }

  public static void intakeOnOutColor(){
    signalLED.set(-0.09); //Blinking Blue Violet
  }

  public static void intakeOnInColor(){
    signalLED.set(-0.07); //Blinking Gold
  }
}
