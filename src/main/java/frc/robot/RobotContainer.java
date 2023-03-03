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

  Trigger button1;
  Trigger button2;
  Trigger button3;
  Trigger button4;
  Trigger button5;
  Trigger button6;
  Trigger button7;
  Trigger button8;
  Trigger button9;
  Trigger button10;
  Trigger button11;
  Trigger button12;

  Telescope m_Telescope;
  Arm m_arm;
  Wrist m_wrist;

  MoveArm moveTo3PtArm;
  MoveArm moveTo2PtArm;
  MoveArm moveTo1PtArm;
  MoveArm moveToRestArm;
  MoveArm moveToHumanIntakeArm;
  MoveArm moveToShelfIntakeArm;
  MoveArmManually moveArmUp;
  MoveArmManually moveArmDown;

  MoveTelescopeTo fullyRetractedTelescope;
  MoveTelescopeTo fullyExtendedTelescope;
  MoveTelescopeTo groundIntakePositionTelescope;
  MoveTelescopeManually extendTelescope;
  MoveTelescopeManually retractTelescope;

  MoveWrist moveTo3PtWrist;
  MoveWrist moveTo2PtWrist;
  MoveWrist moveTo1PtWrist;
  MoveWrist moveToTopWrist;
  MoveWrist moveToShelfIntakeWrist;
  MoveWristManually moveWristUp;
  MoveWristManually moveWristDown;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_arm = new Arm();
    m_Telescope = new Telescope();
    m_wrist = new Wrist();

    moveTo3PtArm = new MoveArm(m_arm, Constants.ArmConstants.Pt3PositionArm);
    moveTo2PtArm = new MoveArm(m_arm, Constants.ArmConstants.Pt2PositionArm);
    moveTo1PtArm = new MoveArm(m_arm, Constants.ArmConstants.Pt1PositionArm);
    moveToRestArm = new MoveArm(m_arm, Constants.ArmConstants.RestPositionArm);
    moveToHumanIntakeArm = new MoveArm(m_arm, Constants.ArmConstants.HumanIntakePositionArm);
    moveToShelfIntakeArm = new MoveArm(m_arm, Constants.ArmConstants.ShelfIntakePositionArm);
    moveArmDown = new MoveArmManually(m_arm, -0.4);
    moveArmUp = new MoveArmManually(m_arm, 0.4);

    fullyRetractedTelescope = new MoveTelescopeTo(m_Telescope, Constants.TelescopeConstants.fullyRetractedPosition);
    fullyExtendedTelescope = new MoveTelescopeTo(m_Telescope, Constants.TelescopeConstants.fullyExtendedPosition);
    groundIntakePositionTelescope = new MoveTelescopeTo(m_Telescope, Constants.TelescopeConstants.groundIntakePosition);
    extendTelescope = new MoveTelescopeManually(m_Telescope, 0.4);
    retractTelescope = new MoveTelescopeManually(m_Telescope, -0.4);

    moveTo3PtWrist = new MoveWrist(m_wrist, Constants.WristConstants.Pt3PositionWrist);
    moveTo2PtWrist = new MoveWrist(m_wrist, Constants.WristConstants.Pt2PositionWrist);
    moveTo1PtWrist = new MoveWrist(m_wrist, Constants.WristConstants.Pt1PositionWrist);
    moveToTopWrist = new MoveWrist(m_wrist, Constants.WristConstants.TopPositionWrist);
    moveToShelfIntakeWrist = new MoveWrist(m_wrist, Constants.WristConstants.ShelfIntakePositionWrist);
    moveWristDown = new MoveWristManually(m_wrist, -0.4);
    moveWristUp = new MoveWristManually(m_wrist, 0.4);

    armJoystick = new Joystick(Constants.ArmJoystickConstants.joystickID);

    Trigger button1 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button1);
    Trigger button2 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button2);
    Trigger button3 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button3);
    Trigger button4 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button4);
    Trigger button5 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button5);
    Trigger button6 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button6);
    Trigger button7 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button7);
    Trigger button8 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button8);
    Trigger button9 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button9);
    Trigger button10 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button10);
    Trigger button11 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button11);
    Trigger button12 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button12);
    
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

    // RESERVED FOR INTAKE AND OUTTAKE
    button1.whileTrue(null);
    button2.whileTrue(null);

    // RESERVED FOR PRESET POSITIONS OF ENTIRE ARM
    button3.onTrue(null);
    button4.onTrue(null);
    button5.onTrue(null);
    button6.onTrue(null);
    button7.onTrue(null);
    button8.onTrue(null);
    button9.onTrue(null);
    button10.onTrue(null);
    button11.onTrue(null);
    button12.onTrue(null);

    m_arm.setArmSpeed(armJoystick.getRawAxis(Constants.ArmJoystickConstants.gripYAxis));
    m_wrist.setWristSpeed(armJoystick.getRawAxis(Constants.ArmJoystickConstants.hatYAxis));
    m_Telescope.moveTelescope(armJoystick.getRawAxis(Constants.ArmJoystickConstants.throttleAxis));
    
    //TELEOP COMMANDS TO TEST
    // moveArmDown
    // moveArmUp
    // moveWristDown
    // moveWristUp

    //AUTON COMMANDS TO TEST
    // fullyRetractedTelescope
    // fullyExtendedTelescope
    // groundIntakePositionTelescope
    // retractTelescope
    // extendTelescope

    // moveTo3PtWrist
    // moveTo2PtWrist
    // moveTo1PtWrist
    // moveToTopWrist
    // moveToShelfIntakeWrist

    // moveTo3PtArm
    // moveTo2PtArm
    // moveTo1PtArm
    // moveToRestArm
    // moveToHumanIntakeArm
    // moveToShelfIntakeArm
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
