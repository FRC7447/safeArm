// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax m_arm;
  DutyCycleEncoder m_armEncoder;
  ProfiledPIDController m_armPID;
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();
  double curAngle;
  ArmFeedforward m_armFeedforward;
  TrapezoidProfile.Constraints m_constraints;
  /** Creates a new arm. */
  public Arm() {
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);

    m_armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    m_armEncoder.setDutyCycleRange(1 / 1025, 1024 / 1025);

    m_armFeedforward = new ArmFeedforward(
      Constants.ArmConstants.armkS, Constants.ArmConstants.armkG,
      Constants.ArmConstants.armkV, Constants.ArmConstants.armkA);
    
    m_constraints = new TrapezoidProfile.Constraints(
      Constants.ArmConstants.trapezoidMaxVelocity, Constants.ArmConstants.trapezoidMaxAcceleration);

    m_armPID = new ProfiledPIDController(
      Constants.ArmConstants.armKP, 
      Constants.ArmConstants.armKI, 
      Constants.ArmConstants.armKD, 
      m_constraints);
  }

  @Override
  public void periodic() {
    curAngle = m_armEncoder.getAbsolutePosition() * 360;

    SmartDashboard.putNumber("Current Absolute Angle: ", curAngle);
    SmartDashboard.putNumber("Arm Applied Output", m_arm.getAppliedOutput());
    SmartDashboard.putNumber("Encoder Offset: ", m_armEncoder.getPositionOffset());
  }

  // Controls a simple motor's position using a SimpleMotorFeedforward
  // and a ProfiledPIDController
  public void goToPosition(double goalPosition) {
    m_armPID.setGoal(goalPosition);
    double pidVal = m_armPID.calculate(curAngle, goalPosition);
    double acceleration = (m_armPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    if(passedLimits()) m_arm.set(pidVal + m_armFeedforward.calculate(m_armPID.getSetpoint().velocity, acceleration));
    else m_arm.set(0);

    lastSpeed = m_armPID.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  public void stop() {
    m_arm.stopMotor();
  }

  public void setArmSpeed(double armSpeed) {
    if(passedLimits()) m_arm.set(0);
    else m_arm.set(armSpeed);
  }

  public boolean passedLimits() {
    if(curAngle >= Constants.ArmConstants.armUpperLimit || curAngle <= Constants.ArmConstants.armLowerLimit) return true;
    else return false;
  }
}