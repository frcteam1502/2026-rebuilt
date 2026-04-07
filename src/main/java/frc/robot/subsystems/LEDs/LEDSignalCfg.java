// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class LEDSignalCfg {

    public static final int LED_PWM_PORT = 9;
    public static final int LED_LENGTH = 60;
    public static final SparkFlex SIGNAL_LED = new SparkFlex(LED_PWM_PORT, MotorType.kBrushless);


}