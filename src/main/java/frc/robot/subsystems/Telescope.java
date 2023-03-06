//system.out.println("Brandyn is the best");
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  GenericHID armJoystick;
  TalonSRX m_telescope;
  Encoder telescopeEncoder;
  double goal = 0.0;

  Faults m_faults = new Faults();
  /** Creates a new telescope. */
  public Telescope() {
    armJoystick = new GenericHID(Constants.ArmJoystickConstants.joystickID);

    telescopeEncoder = new Encoder(1, 2);
    m_telescope = new TalonSRX(Constants.TelescopeConstants.telescopeID);

    // Jason's Code
    // m_telescope.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 1, 30);
    // m_telescope.configSelectedFeedbackSensor(telescopeEncoder);
    // m_telescope.configNeutralDeadband(0.001, 30);
    // m_telescope.setInverted(false);
    // m_telescope.setInverted(false);

    // m_telescope.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    // m_telescope.configNominalOutputForward(0, 30);
    // m_telescope.configNominalOutputReverse(-1, 30);
    // m_telescope.configPeakOutputForward(1, 30);
    // m_telescope.configPeakOutputReverse(-1, 30);

    // m_telescope.selectProfileSlot(0, 0);
    // m_telescope.config_kF(0, 0.2, 30);
    // m_telescope.config_kP(0, 0.2, 30);
    // m_telescope.config_kI(0, 0.0, 30);
    // m_telescope.config_kD(0, 0.0, 30);

    // m_telescope.configMotionCruiseVelocity(15000, 30);
    // m_telescope.configMotionAcceleration(6000, 30);
    // m_telescope.setSelectedSensorPosition(0, 0, 30);

    
    m_telescope.configFactoryDefault(0);
    // m_telescope.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_telescope.setInverted(false);
    
    m_telescope.setNeutralMode(NeutralMode.Brake);
    // // m_telescope.configNeutralDeadband(0.001); // 0.1%
    // // m_telescope.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    // // m_telescope.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)
    // // m_telescope.setSensorPhase(false); // May need to swap!!!
    
    // m_telescope.config_kP(0, Constants.TelescopeConstants.telescopeKP);
    // m_telescope.config_kI(0, Constants.TelescopeConstants.telescopeKI);
    // m_telescope.config_kD(0, Constants.TelescopeConstants.telescopeKD);

    // // m_telescope.configVoltageCompSaturation(12); // "full output" will now scale to x volts
    // // m_telescope.enableVoltageCompensation(true); // turn on/off feature

    // m_telescope.configForwardSoftLimitEnable(true, 10000000);
    // m_telescope.configReverseSoftLimitEnable(true, 10000000);
    // m_telescope.configForwardSoftLimitThreshold(Constants.TelescopeConstants.telescopeUpperLimit, 100000);
    // m_telescope.configReverseSoftLimitThreshold(Constants.TelescopeConstants.telescopeLowerLimit, 1000000);

    resetTelescope();

    // FOR TUNING
    // SmartDashboard.putNumber("Telescope P Gain: ", Constants.TelescopeConstants.telescopeKP);
    // SmartDashboard.putNumber("Telescope I Gain: ", Constants.TelescopeConstants.telescopeKI);
    // SmartDashboard.putNumber("Telescope D Gain: ", Constants.TelescopeConstants.telescopeKD);
    SmartDashboard.putNumber("Telescope Upper Limit: ", Constants.TelescopeConstants.telescopeUpperLimit);
    SmartDashboard.putNumber("Telescope Lower Limit: ", Constants.TelescopeConstants.telescopeLowerLimit);
    SmartDashboard.putBoolean("Enable Telescope Limits: ", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Telescope Encoder Position: ", telescopeEncoder.getRaw());
    this.moveTelescope(armJoystick.getRawAxis(Constants.ArmJoystickConstants.throttleAxis));
    m_telescope.getFaults(m_faults);

    // FOR TUNING
    // double kP = SmartDashboard.getNumber("Telescope P Gain: ", Constants.TelescopeConstants.telescopeKP);
    // double kI = SmartDashboard.getNumber("Telescope I Gain: ", Constants.TelescopeConstants.telescopeKI);
    // double kD = SmartDashboard.getNumber("Telescope D Gain: ", Constants.TelescopeConstants.telescopeKD);
    double topLimit = SmartDashboard.getNumber("Telescope Upper Limit: ", Constants.TelescopeConstants.telescopeUpperLimit);
    double botLimit = SmartDashboard.getNumber("Telescope Lower Limit: ", Constants.TelescopeConstants.telescopeLowerLimit);
    boolean eLimits = SmartDashboard.getBoolean("Enable Telescope Limits: ", false);

    /*m_telescope.configForwardSoftLimitThreshold(topLimit, 10000);
    m_telescope.configReverseSoftLimitThreshold(botLimit, 100000);
    m_telescope.configForwardSoftLimitEnable(eLimits, 0);
    m_telescope.configReverseSoftLimitEnable(eLimits, 0);
    m_telescope.config_kP(0, kP);
    m_telescope.config_kI(0, kI);
    m_telescope.config_kD(0, kD);

    SmartDashboard.putNumber("Telescope Goal Position: ", goal);
    SmartDashboard.putNumber("Telescope Position: ", m_telescope.getSelectedSensorPosition());
    // This method will be called once per scheduler run*/
  }

  public void moveTelescope(double telescopeSpeed) {
    if(telescopeSpeed == 1.0) {
      m_telescope.set(TalonSRXControlMode.PercentOutput, Constants.TelescopeConstants.telescopeSpeed);
    //  m_telescope.set(TalonSRXControlMode.MotionMagic, armJoystick.getRawAxis(Constants.ArmJoystickConstants.throttleAxis));
    } else if(telescopeSpeed == -1.0) {
      // m_telescope.set(TalonSRXControlMode.PercentOutput, armJoystick.getRawAxis(-Constants.ArmJoystickConstants.throttleAxis));
      m_telescope.set(TalonSRXControlMode.PercentOutput, -Constants.TelescopeConstants.telescopeSpeed);
    } else {
      m_telescope.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  public void extendTelescopeTo(double desiredPosition) {
    goal = desiredPosition;
    m_telescope.set(TalonSRXControlMode.Position, desiredPosition);
  }

  public void stopTelescope() {
    m_telescope.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public void resetTelescope() {
    m_telescope.setSelectedSensorPosition(0.0, 0, 0);
  }

  // public boolean getTopSwitch() {
  //   if(telescopeEncoder.getDistance() >= Constants.TelescopeConstants.telescopeUpperLimit) {
  //     return true;
  //   }

  //   return false;
  // }

  // public boolean getBottomSwitch() {
  //   if(telescopeEncoder.getDistance() <= Constants.TelescopeConstants.telescopeLowerLimit) {
  //     return true;
  //   } 

  //   return false;
  // }
}