// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterCfg;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestTurret extends Command {
  /** Creates a new TestTurret. */
  Shooter shooter;
  Timer testTimer;

  private enum TurretTestState{
    NOT_STARTED,
    TEST_MIN,
    DELAY1,
    TEST_MAX,
    DELAY2,
    CENTER,
    FINISHED;
  }

  private boolean isTestPassed = false;
  private boolean isTestStarted = false;
  private boolean isTestComplete = false;

  private final double TEST_TIME = 3.0;
  private final double TEST_DELAY = 1.0;

  private TurretTestState testState = TurretTestState.NOT_STARTED;

  public TestTurret(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;

    SmartDashboard.putBoolean("Turret Test Started", isTestStarted);
    SmartDashboard.putBoolean("Turret Test Passed", isTestPassed);
    SmartDashboard.putBoolean("Turret Test Complete", isTestComplete);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTestStarted = true;
    shooter.setTurretAngle(ShooterCfg.TURRET_MIN_ANGLE + Math.toRadians(5));
    testTimer.start();
    testState = TurretTestState.TEST_MIN;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(testState){
      case NOT_STARTED:
        //Should never get here
        break;
      case TEST_MIN:
        if(shooter.isTurretAtSetpoint()){
          //We hit the set point, proceed to the next step
          testTimer.reset();
          testState = TurretTestState.DELAY1;
        }else if(testTimer.get() >= TEST_TIME){
          //Test timed out so finish with a fail
          isTestPassed = false;
          isTestComplete = true;
        }else{
          //Wait for test to complete or timeout
        }
        break;
      case DELAY1:
        if(testTimer.get() >= TEST_DELAY){
          shooter.setTurretAngle(ShooterCfg.TURRET_MAX_ANGLE - Math.toRadians(5));
          testTimer.reset();
          testState = TurretTestState.TEST_MAX;
        }else{
          //Wait
        }
        break;
      case TEST_MAX:
        if(shooter.isTurretAtSetpoint()){
          //We hit the set point, proceed to the next step
          testTimer.reset();
          testState = TurretTestState.DELAY2;
        }else if(testTimer.get() >= TEST_TIME){
          //Test timed out so finish with a fail
          isTestPassed = false;
          isTestComplete = true;
        }else{
          //Wait for test to complete or timeout
        }
        break;
      case DELAY2:
        if(testTimer.get() >= TEST_DELAY){
          shooter.setTurretAngle(Math.toRadians(180));
          testTimer.reset();
          testState = TurretTestState.CENTER;
        }else{
          //Wait
        }
        break;
      case CENTER:
        if(shooter.isTurretAtSetpoint()){
          //We hit the set point, proceed to the next step
          testTimer.stop();
          isTestPassed = true;
          isTestComplete = true;
          testState = TurretTestState.FINISHED;
        }else if(testTimer.get() >= TEST_TIME){
          //Test timed out so finish with a fail
          isTestPassed = false;
          isTestComplete = true;
        }else{
          //Wait for test to complete or timeout
        }
        break;
      case FINISHED:
        //Do nothing
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isTestComplete){
      return true;
    }
    return false;
  }
}
