// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmManually;
import frc.robot.commands.MoveClawAuton;
import frc.robot.commands.MoveClawManually;
import frc.robot.commands.MoveTelescopeManually;
import frc.robot.commands.MoveTelescopeTo;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.MoveWristManually;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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

  Trigger POV0button;
  Trigger POV180button;

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
  Claw m_claw;

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

  MoveClawManually intake;
  MoveClawManually outtake;
  MoveClawAuton intake_auton;
  MoveClawAuton outtake_auton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_arm = new Arm();
    m_Telescope = new Telescope();
    m_wrist = new Wrist();
    m_claw = new Claw();

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

    intake = new MoveClawManually(m_claw, -Constants.ClawConstants.clawSpeed);
    outtake = new MoveClawManually(m_claw, Constants.ClawConstants.clawSpeed);
    intake_auton = new MoveClawAuton(m_claw, false);
    outtake_auton = new MoveClawAuton(m_claw, true);

    armJoystick = new Joystick(Constants.ArmJoystickConstants.joystickID);

    POV0button = new POVButton(armJoystick, Constants.ArmJoystickConstants.buttonPOV0);
    POV180button = new POVButton(armJoystick, Constants.ArmJoystickConstants.buttonPOV180);

    button1 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button1);
    button2 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button2);
    button3 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button3);
    button4 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button4);
    button5 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button5);
    button6 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button6);
    button7 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button7);
    button8 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button8);
    button9 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button9);
    button10 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button10);
    button11 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button11);
    button12 = new JoystickButton(armJoystick, Constants.ArmJoystickConstants.button12);
    
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

    POV0button.whileTrue(moveWristUp);
    POV180button.whileTrue(moveWristDown);
    // m_wrist.setWristSpeed(armJoystick.getRawAxis(Constants.ArmJoystickConstants.hatYAxis))

    // RESERVED FOR INTAKE AND OUTTAKE
    button1.whileTrue(intake);
    button2.whileTrue(outtake);

    // RESERVED FOR PRESET POSITIONS OF ENTIRE ARM
    // Right hand buttons
    button3.onTrue(null);
    button4.onTrue(null);
    button5.onTrue(null);
    button6.onTrue(null);

    // Left hand buttons
    button7.onTrue(null);
    button8.onTrue(null);
    button9.onTrue(null);
    button10.onTrue(null);
    button11.onTrue(null);
    button12.onTrue(null);

    m_arm.setArmSpeed(armJoystick.getRawAxis(Constants.ArmJoystickConstants.gripYAxis));
    m_Telescope.moveTelescope(armJoystick.getRawAxis(Constants.ArmJoystickConstants.throttleAxis));

    //AUTON COMMANDS TO TEST
    // fullyRetractedTelescope
    // fullyExtendedTelescope
    // groundIntakePositionTelescope

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

    // intake_auton
    // outtake_auton
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
