// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.PowerManagement.MockDetector;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.commands.AlignToTowerLeft;
import frc.robot.commands.AlignToTowerRight;
import frc.robot.commands.AlignToTowerLeft;
import frc.robot.commands.AlignToTowerRight;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.ExtendAndAlignLeft;
import frc.robot.commands.ExtendAndAlignRight;
import frc.robot.commands.OperatorCommands;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.StopDriveMotors;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Intake intake = new Intake();
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final Shooter shooter = new Shooter(driveSubsystem, intake);
  //private final PdpSubsystem pdpSubsystem = new PdpSubsystem();
  public final Climber climber = new Climber(driveSubsystem);


  private final SendableChooser<Command> autoChooser; 

  /* sample

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  */

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Register named commands. Must register all commands we want Pathplanner to execute.
   
    NamedCommands.registerCommand("climberExtend", new InstantCommand(climber::setClimberOut));
    NamedCommands.registerCommand("climberRetract", new InstantCommand(climber::setClimberIn));

    NamedCommands.registerCommand("extendIntake", new InstantCommand(intake::setHopperOut));
    NamedCommands.registerCommand("retractIntake", new InstantCommand(intake::setHopperIn));

    NamedCommands.registerCommand("intakeOff", new InstantCommand(intake::setIntakeOff));
    NamedCommands.registerCommand("intakeOn", new InstantCommand(intake::setIntakeOn));
    NamedCommands.registerCommand("intakeReverse", new InstantCommand(intake::setIntakeReverse));

    NamedCommands.registerCommand("shoot", new AutoShoot(shooter));
    NamedCommands.registerCommand("shootOn", new InstantCommand(shooter::setShooterOn));
    NamedCommands.registerCommand("shootOff", new InstantCommand(shooter::setShooterOff));
    NamedCommands.registerCommand("Stop Drive Motors", new StopDriveMotors(driveSubsystem));

    NamedCommands.registerCommand("alignToClimbRight", new ExtendAndAlignRight(driveSubsystem, climber));
    NamedCommands.registerCommand("alignToClimbLeft", new ExtendAndAlignLeft(driveSubsystem, climber));

    

    //Build an Autochooser from SmartDashboard selection.  Default will be Commands.none()
    //e.g new PathPlannerAuto("MiddleAutoAMPFinal");
    //Left Start
    //new PathPlannerAuto("LeftCenterGrab");
    //new PathPlannerAuto("LeftCenterShoot");
    //Right Start
    //new PathPlannerAuto("RightCenterGrab");
    //new PathPlannerAuto("RightCenterShoot");
    //Center Start
    //new PathPlannerAuto("CenterStartGHP");
    //new PathPlannerAuto("GetOutOfTheWay");
    //new PathPlannerAuto("CenterStartGround");
    //new PathPlannerAuto("CenterStartHp");
    new PathPlannerAuto("LeftWeek0CenterGrab");

    new PathPlannerAuto("TestAuto");
    new PathPlannerAuto("StrafeTestAuto");
   
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Drivetrain
    driveSubsystem.setDefaultCommand(new DriverCommands(driveSubsystem, 
                                                        new MockDetector(),
                                                        ()->{ return false;})); //USES THE Right BUMPER TO SLOW DOWN

    Driver.Controller.start().onTrue(new ResetGyro(driveSubsystem));
    Driver.Controller.y().onTrue(new InstantCommand(climber::toggleClimber));
    Driver.Controller.x().whileTrue(new ExtendAndAlignLeft(driveSubsystem, climber));
    Driver.Controller.b().whileTrue(new ExtendAndAlignRight(driveSubsystem, climber));
    
    //shooter.setDefaultCommand(new OperatorCommands(shooter));
    //Operator.Controller.rightStick().onTrue(new InstantCommand(shooter::toggleAutoAim));
    //Operator.Controller.leftStick().onTrue(new InstantCommand(shooter::toggleHoodAim));
    Operator.Controller.start().onTrue(new InstantCommand(shooter::toggleTestMode));

    Operator.Controller.leftTrigger().whileTrue(new InstantCommand(intake::setIntakeOn)).onFalse(new InstantCommand(intake::setIntakeOff));
    Operator.Controller.leftBumper().whileTrue(new InstantCommand(intake::setIntakeReverse)).onFalse(new InstantCommand(intake::setIntakeOff));;
    Operator.Controller.rightTrigger().whileTrue(new InstantCommand(shooter::setShooterOn)).onFalse(new InstantCommand(shooter::setShooterToWait));
    Operator.Controller.a().onTrue(new InstantCommand(intake::toggleHopper));

    //Operator.Controller.x().whileTrue(new InstantCommand(shooter::setFeedOn)).whileFalse(new InstantCommand(shooter::setFeedOff));
    //Operator.Controller.b().whileTrue(new InstantCommand(shooter::setIndexerOn)).whileFalse(new InstantCommand(shooter::setIndexerOff));
    
    //Drive SysID stuff - comment out on competition build!
    /*Driver.Controller.y().whileTrue(driveSubsystem.sysIdLinearQuasistatic(Direction.kForward));
    Driver.Controller.a().whileTrue(driveSubsystem.sysIdLinearQuasistatic(Direction.kReverse));
    Driver.Controller.b().whileTrue(driveSubsystem.sysIdLinearDynamic(Direction.kForward));
    Driver.Controller.x().whileTrue(driveSubsystem.sysIdLinearDynamic(Direction.kReverse));*/
    
    //Shooter SysID stuff - comment out on competition build!
    /*Operator.Controller.rightTrigger().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    Operator.Controller.leftTrigger().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
    Operator.Controller.rightBumper().whileTrue(shooter.sysIdDynamic(Direction.kForward));
    Operator.Controller.leftBumper().whileTrue(shooter.sysIdDynamic(Direction.kReverse));*/

    /* sample code
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
