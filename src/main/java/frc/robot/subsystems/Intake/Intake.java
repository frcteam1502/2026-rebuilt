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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
      updateSmartDashboard();
      arbitrateIntake();
      //SmartDashboard.putData(LEDSignals.signalLED);
    }

    void updateSmartDashboard(){
      SmartDashboard.putNumber("Intake Output Current", leadIntakeMotor.getOutputCurrent());
      SmartDashboard.putNumber("Intake Velocity", leadIntakeMotor.get());
    }

    private void arbitrateIntake(){
      if(isCntrlRequestIn){
        leadIntakeMotor.set(1.0);
        if(hopperIn){
         // LEDSignals.();
        }else if(hopperIn==false){
          LEDSignals.intakeOnOutColor();
        };
      }else if(isCntrlRequestOut){
        leadIntakeMotor.set(-1.0);
        if(hopperIn){
          LEDSignals.intakeOnInColor();
        }else if(hopperIn==false){
          LEDSignals.intakeOnOutColor();
        };
        LEDSignals.hopperOutColor();
      }else if(isShooterRequestIn){
        leadIntakeMotor.set(1.0);
        if(hopperIn){
          LEDSignals.intakeOnInColor();
        }else if(hopperIn==false){
          LEDSignals.intakeOnOutColor();
        };
      }else if(isShooterRequestSlow){
        leadIntakeMotor.set(0.5);
        if(hopperIn){
          LEDSignals.intakeOnInColor();
        }else if(hopperIn==false){
          LEDSignals.intakeOnOutColor();
        };
      }else if(isShooterRequestOut){
        leadIntakeMotor.set(-1.0);
        if(hopperIn){
          LEDSignals.intakeOnInColor();
        }else if(hopperIn==false){
          LEDSignals.intakeOnOutColor();
        };
      }else{
        leadIntakeMotor.set(0);
        if(hopperIn){
          LEDSignals.hopperInColor();
        }else if(hopperIn==false){
          LEDSignals.hopperOutColor();
        };
      }
    }
    
    
    public void setIntakeOn(){
      System.out.println("Intake ON");
      isCntrlRequestIn = true;
      isCntrlRequestOut = false;
    }
  
    public void setIntakeReverse(){
      isCntrlRequestIn = false;
      isCntrlRequestOut = true;
    }
    
    public void setIntakeOff(){
      System.out.println("Intake OFF");
      isCntrlRequestIn = false;
      isCntrlRequestOut = false;
      setIntakeSpeed(0);
    }
  
    public void setHopperOut(){
      System.out.println("Hopper OUT");
      hopperSolenoid.set(true);
      hopperIn = false;
    }
    public void setHopperIn(){
      System.out.println("Hopper IN");
      hopperIn = true;
      hopperSolenoid.set(false);
      isCntrlRequestOut = false;
    }
    public void setIntakeSpeed(double speed){
      leadIntakeMotor.set(speed);
    }
  
  public void shooterRequestIntakeOn(){
    System.out.println("Intake ON");
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
    System.out.println("Intake OFF");
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

  public Command systemsCheckIntakeCommand() {
      return Commands.sequence(
        setHopperOutCommand(),
        Commands.waitSeconds(2),
        setIntakeOnCommand(),
        Commands.waitSeconds(4),
        setHopperInCommand(),
        Commands.waitSeconds(2),
        setIntakeOffCommand(),
        Commands.waitSeconds(2)
      ); 
  }

  public Command setHopperOutCommand(){
    return this.runOnce(this::setHopperOut);
  }
  public Command setHopperInCommand(){
    return this.runOnce(this::setHopperIn);
  }

  public Command setIntakeOnCommand(){
    return this.runOnce(this::setIntakeOn);
  }
  public Command setIntakeOffCommand(){
    return this.runOnce(this::setIntakeOff);
  }
 }
