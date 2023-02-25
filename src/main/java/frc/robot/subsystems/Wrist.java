// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  VictorSPX m_wrist;
  CANSparkMax m_intake;
  DutyCycleEncoder m_wristEncoder;

  /** Creates a new wrist. */
  public Wrist() {
    m_wrist = new VictorSPX(Constants.ArmConstants.wristID);
    m_intake = new CANSparkMax(Constants.ArmConstants.intakeID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
