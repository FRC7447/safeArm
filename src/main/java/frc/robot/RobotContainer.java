// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmManually;
import frc.robot.commands.MoveTelescopeManually;
import frc.robot.commands.MoveTelescopeTo;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.MoveWristManually;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;
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
  Joystick armJoystick;

  Trigger lTrigger;  
  Trigger rTrigger;
  Trigger lTab;
  Trigger rTab;

  Telescope m_Telescope;
  Arm m_arm;
  Wrist m_wrist;

  MoveArm moveTo3PtArm;
  MoveArm moveTo2PtArm;
  MoveArm moveTo1PtArm;
  MoveArm moveToRestArm;
  MoveArmManually moveArmUp;
  MoveArmManually moveArmDown;

  MoveTelescopeTo fullyRetracted;
  MoveTelescopeTo fullyExtended;
  MoveTelescopeTo groundIntakePos;
  MoveTelescopeManually extendTelescope;
  MoveTelescopeManually retractTelescope;

  MoveWrist moveTo3PtWrist;
  MoveWrist moveTo2PtWrist;
  MoveWrist moveTo1PtWrist;
  MoveWrist moveToTopWrist;
  MoveWristManually moveWristUp;
  MoveWristManually moveWristDown;
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_arm = new Arm();
    m_Telescope = new Telescope();
    m_wrist = new Wrist();

    moveTo3PtArm = new MoveArm(m_arm, Constants.ArmConstants.Pt3PositionArm);
    moveTo2PtArm = new MoveArm(m_arm, Constants.ArmConstants.Pt2PositionArm);
    moveTo1PtArm = new MoveArm(m_arm, Constants.ArmConstants.Pt1PositionArm);
    moveToRestArm = new MoveArm(m_arm, Constants.ArmConstants.RestPositionArm);
    moveArmDown = new MoveArmManually(m_arm, Constants.ArmConstants.downArmSpeed);
    moveArmUp = new MoveArmManually(m_arm, Constants.ArmConstants.upArmSpeed);

    fullyRetracted = new MoveTelescopeTo(m_Telescope, Constants.TelescopeConstants.fullyRetractedPosition);
    fullyExtended = new MoveTelescopeTo(m_Telescope, Constants.TelescopeConstants.fullyExtendedPosition);
    groundIntakePos = new MoveTelescopeTo(m_Telescope, Constants.TelescopeConstants.groundIntakePosition);
    extendTelescope = new MoveTelescopeManually(m_Telescope, true);
    retractTelescope = new MoveTelescopeManually(m_Telescope, false);

    moveTo3PtWrist = new MoveWrist(m_wrist, Constants.WristConstants.Pt3PositionWrist);
    moveTo2PtWrist = new MoveWrist(m_wrist, Constants.WristConstants.Pt2PositionWrist);
    moveTo1PtWrist = new MoveWrist(m_wrist, Constants.WristConstants.Pt1PositionWrist);
    moveToTopWrist = new MoveWrist(m_wrist, Constants.WristConstants.TopPositionWrist);
    moveWristDown = new MoveWristManually(m_wrist, Constants.WristConstants.downWristSpeed);
    moveWristUp = new MoveWristManually(m_wrist, Constants.WristConstants.upWristSpeed);

    armJoystick = new Joystick(Constants.ArmJoystickConstants.joystickID);
    //limelight = new Limelight();
    lTrigger = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.lTrigger);
    rTrigger = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.rTrigger);
    lTab = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.lTab);
    rTab = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.rTab);

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

    lTrigger.onTrue(extendTelescope);
    rTrigger.onTrue(retractTelescope);
    lTab.onTrue(moveTo3PtArm);
    rTab.onTrue(moveTo1PtArm);
    
    //TELEOP COMMANDS TO TEST
    // moveArmDown
    // moveArmUp
    // moveWristDown
    // moveWristUp

    //AUTON COMMANDS TO TEST
    // fullyRetracted
    // fullyExtended
    // groundIntakePos

    // moveTo3PtWrist
    // moveTo2PtWrist
    // moveTo1PtWrist
    // moveToTopWrist

    // moveTo2PtArm
    // moveToRestArm
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
