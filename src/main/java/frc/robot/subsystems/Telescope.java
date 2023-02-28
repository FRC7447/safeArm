//system.out.println("Brandyn is the best");
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  TalonSRX m_telescope;
  Encoder telescopeEncoder;

  Faults m_faults = new Faults();
  /** Creates a new telescope. */
  public Telescope() {
    m_telescope = new TalonSRX(Constants.TelescopeConstants.telescopeID);
    m_telescope.configFactoryDefault(0);
    m_telescope.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    
    m_telescope.setNeutralMode(NeutralMode.Brake);
    m_telescope.configNeutralDeadband(0.001); // 0.1%
    m_telescope.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    m_telescope.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)
    m_telescope.setSensorPhase(false); // May need to swap!!!
    
    m_telescope.config_kP(0, Constants.TelescopeConstants.telescopeKP);
    m_telescope.config_kI(0, Constants.TelescopeConstants.telescopeKI);
    m_telescope.config_kD(0, Constants.TelescopeConstants.telescopeKD);

    m_telescope.configVoltageCompSaturation(12); // "full output" will now scale to x volts
    m_telescope.enableVoltageCompensation(true); // turn on/off feature

    m_telescope.configForwardSoftLimitThreshold(Constants.TelescopeConstants.telescopeUpperLimit, 0);
    m_telescope.configReverseSoftLimitThreshold(Constants.TelescopeConstants.telescopeLowerLimit, 0);
    m_telescope.configForwardSoftLimitEnable(false, 0);
    m_telescope.configReverseSoftLimitEnable(false, 0);

    resetTelescope();
  }

  @Override
  public void periodic() {
    m_telescope.getFaults(m_faults);
    System.out.println("Sensor Vel:" + m_telescope.getSelectedSensorVelocity());
    System.out.println("Sensor Pos:" + m_telescope.getSelectedSensorPosition());
    System.out.println("Out %" + m_telescope.getMotorOutputPercent());
    System.out.println("Out Of Phase:" + m_faults.SensorOutOfPhase);
    // This method will be called once per scheduler run
  }

  public void moveTelescope(boolean lDirection) {
    if(lDirection) {
      m_telescope.set(TalonSRXControlMode.PercentOutput, 1.0 * Constants.TelescopeConstants.telescopeSpeed);
    } else {
      m_telescope.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.TelescopeConstants.telescopeSpeed);
    }
  }

  public void extendTelescopeTo(double desiredPosition) {
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
