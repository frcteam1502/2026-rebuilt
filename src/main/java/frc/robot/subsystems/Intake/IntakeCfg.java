package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeCfg {
//Intake Motor Config
public static final int LEAD_INTAKE_MOTOR_ID = 3;
public static final int FOLLOWER_INTAKE_MOTOR_ID = 99;

public static final int INTAKE_SOLENOID_ID = 0;

public static final int FRONT_HOPPER_SENSOR_ID = 1;
public static final int REAR_HOPPER_SENSOR_ID = 2;

public static final SparkFlex LEAD_INTAKE_MOTOR = new SparkFlex(LEAD_INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
public static final SparkFlex FOLLOWER_INTAKE_MOTOR = new SparkFlex(FOLLOWER_INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
public static final Solenoid INTAKE_SOLENOID = new Solenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_ID);

//public static final LaserCan FRONT_LASER_CAN = new LaserCan(FRONT_HOPPER_SENSOR_ID);
//public static final LaserCan REAR_LASER_CAN =  new LaserCan(REAR_HOPPER_SENSOR_ID);

public static final boolean LEAD_INTAKE_MOTOR_REVERSED = true;
public static final boolean FOLLOWER_INTAKE_MOTOR_REVERSED = true;

public static final IdleMode INTAKE_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;

public static final int INTAKE_MOTOR_CURRENT_LIMIT = 80;



}
