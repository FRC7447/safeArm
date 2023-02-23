// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.extendTelescope;
import frc.robot.commands.moveArm;
import frc.robot.commands.retractTelescope;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Joystick driver;

  Trigger lTrigger;  
  Trigger rTrigger;
  Trigger lTab;
  Trigger rTab;

  Telescope m_Telescope;
  Arm m_arm;

  moveArm moveTo3Pt;
  moveArm moveTo2Pt;
  moveArm moveTo1Pt;
  moveArm moveToRest;
  extendTelescope m_ExtendTelescope;
  retractTelescope m_RetractTelescope;
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_arm = new Arm();
    m_Telescope = new Telescope();
    moveTo3Pt = new moveArm(m_arm, Constants.Pt3Position);
    moveTo2Pt = new moveArm(m_arm, Constants.Pt2Position);
    moveTo1Pt = new moveArm(m_arm, Constants.Pt1Position);
    moveToRest = new moveArm(m_arm, Constants.RestPosition);
    m_ExtendTelescope = new extendTelescope(m_Telescope);
    m_RetractTelescope = new retractTelescope(m_Telescope);

    driver = new Joystick(0);
    //limelight = new Limelight();
    lTrigger = new JoystickButton(driver, Constants.lTrigger);
    rTrigger = new JoystickButton(driver, Constants.rTrigger);
    lTab = new JoystickButton(driver, Constants.lTab);
    rTab = new JoystickButton(driver, Constants.rTab);

    // Configure the trigger bindings
    configureBindings();
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    lTrigger.onTrue(m_ExtendTelescope);
    rTrigger.onTrue(m_RetractTelescope);
    lTab.onTrue(moveTo3Pt);
    rTab.onTrue(moveTo1Pt);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; 
    // Autos.exampleAuto(m_exampleSubsystem);
  }
}
