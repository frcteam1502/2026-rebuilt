// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class LEDSignalCfg {

    public static final int LED_PWM_PORT = 0;
    public static final int LED_LENGTH = 60;
    public static final Spark SIGNAL_LED = new Spark(LED_PWM_PORT);
    CANdle CANDLE = new CANdle(14, "light bus");

}