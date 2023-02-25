// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  VictorSPX m_wrist;
  CANSparkMax m_intake;
  DutyCycleEncoder m_wristEncoder;
  PIDController wristPID; 

  double goal = 0.0;
  double encoderPos = 0.0;
  double pidOut = 0.0;
  double output = 0.0;
  
  /** Creates a new wrist. */
  public Wrist() {
    m_wrist = new VictorSPX(Constants.WristConstants.wristID);
    wristPID = new PIDController(
      Constants.WristConstants.wristkP, 
      Constants.WristConstants.wristkI, 
      Constants.WristConstants.wristkD
    );
    // m_intake = new CANSparkMax(Constants.ArmConstants.intakeID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    encoderPos = m_wristEncoder.getAbsolutePosition();
    pidOut = wristPID.calculate(m_wristEncoder.getAbsolutePosition(), goal);
    output = pidOut;
    // This method will be called once per scheduler run
    if (encoderPos > Constants.ArmConstants.armUpperLimit) {
      output = 0.0;
      stop();
    } else if (encoderPos < Constants.ArmConstants.armLowerLimit) {
      output = 0.0;
      stop();
    }

    m_wrist.set(ControlMode.PercentOutput, output);
    // m_wrist.configForwardSoftLimitThreshold();

    if(m_wristEncoder.getAbsolutePosition() == goal) {
      output = 0.0;
      stop();
    }
  }

  public void stop() {
    m_wrist.set(ControlMode.PercentOutput, 0.0);
  }

  public void setGoal(double desiredPosition) {
    goal = desiredPosition;
  }

  public void setWristSpeed(double wristSpeed) {
    m_wrist.set(ControlMode.PercentOutput, wristSpeed);
  }
}
