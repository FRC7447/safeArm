// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  TalonSRX m_telescope;
  Encoder telescopeEncoder;
  /** Creates a new telescope. */
  public Telescope() {
    m_telescope = new TalonSRX(Constants.TelescopeConstants.telescopeID);
    telescopeEncoder = new Encoder(0, 1);
    telescopeEncoder.reset();
    telescopeEncoder.setDistancePerPulse(8.0);

    m_telescope.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("telescope encoder: ", m_telescope.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  public void moveTelescope(boolean lDirection) {
    if(lDirection) {
      m_telescope.set(TalonSRXControlMode.PercentOutput, 1.0 * Constants.TelescopeConstants.telescopeSpeed);
    } else {
      m_telescope.set(TalonSRXControlMode.PercentOutput, -1.0 * Constants.TelescopeConstants.telescopeSpeed);
    }
  }

  public void stopTelescope() {
    m_telescope.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public boolean getTopSwitch() {
    if(telescopeEncoder.getDistance() >= Constants.TelescopeConstants.telescopeUpperLimit) {
      return true;
    }

    return false;
  }

  public boolean getBottomSwitch() {
    if(telescopeEncoder.getDistance() <= Constants.TelescopeConstants.telescopeLowerLimit) {
      return true;
    } 

    return false;
  }
}
