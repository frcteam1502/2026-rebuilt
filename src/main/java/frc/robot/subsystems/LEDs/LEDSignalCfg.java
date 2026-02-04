// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/** Add your docs here. */
public class LEDSignalCfg {

    public static final int LED_PWM_PORT = 9;
    public static final int LED_LENGTH = 60;

     public static final Spark SIGNAL_LED = new Spark(LED_PWM_PORT);


}