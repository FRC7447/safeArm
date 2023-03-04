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
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax m_arm;
  DutyCycleEncoder m_armEncoder;
  ProfiledPIDController m_armPID;
  double goal; 
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();
  double curAngle;
  double armTopLimit;
  double armBotLimit;
  ArmFeedforward m_armFeedforward;
  TrapezoidProfile.Constraints m_constraints;
  /** Creates a new arm. */
  public Arm() {
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);
    m_arm.enableVoltageCompensation(12);
    m_arm.setInverted(false);

    m_armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    // new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    m_armEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

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
    
    armTopLimit = Constants.ArmConstants.armUpperLimit;
    armBotLimit = Constants.ArmConstants.armLowerLimit;
    SmartDashboard.putNumber("Arm Top Limit", Constants.ArmConstants.armUpperLimit);
    SmartDashboard.putNumber("Arm Bot Limit", Constants.ArmConstants.armLowerLimit);
    // FOR TUNING
    SmartDashboard.putNumber("Arm P Gain", Constants.ArmConstants.armKP);
    SmartDashboard.putNumber("Arm I Gain", Constants.ArmConstants.armKI);
    SmartDashboard.putNumber("Arm D Gain", Constants.ArmConstants.armKD);
    SmartDashboard.putNumber("Arm kG Gain", Constants.ArmConstants.armkG);
    SmartDashboard.putNumber("Arm kV Gain", Constants.ArmConstants.armkV);
    SmartDashboard.putNumber("Arm Trapezoid Max Velocity", Constants.ArmConstants.trapezoidMaxVelocity);
    SmartDashboard.putNumber("Arm Trapezoid Max Acceleration", Constants.ArmConstants.trapezoidMaxAcceleration);
  }

  @Override
  public void periodic() {
    curAngle = m_armEncoder.getAbsolutePosition() * 360;

    SmartDashboard.putNumber("Arm Goal Setpoint: ", goal);
    SmartDashboard.putNumber("Arm Current Absolute Angle: ", curAngle);
    SmartDashboard.putNumber("Arm Applied Output", m_arm.getAppliedOutput());
    SmartDashboard.putNumber("Arm Encoder Offset: ", m_armEncoder.getPositionOffset());

    // FOR VALE TUNING
    armTopLimit = SmartDashboard.getNumber("Arm Top Limit", Constants.ArmConstants.armUpperLimit);
    armBotLimit = SmartDashboard.getNumber("Arm Bot Limit", Constants.ArmConstants.armLowerLimit);

    m_armPID.setP(SmartDashboard.getNumber("Arm P Gain", Constants.ArmConstants.armKP));
    m_armPID.setI(SmartDashboard.getNumber("Arm I Gain", Constants.ArmConstants.armKI));
    m_armPID.setD(SmartDashboard.getNumber("Arm D Gain", Constants.ArmConstants.armKD));
    m_armFeedforward = new ArmFeedforward(
      Constants.ArmConstants.armkS, SmartDashboard.getNumber("Arm kG Gain", Constants.ArmConstants.armkG),
      SmartDashboard.getNumber("Arm kV Gain", Constants.ArmConstants.armkV), Constants.ArmConstants.armkA);
    m_constraints = new TrapezoidProfile.Constraints(
      SmartDashboard.getNumber("Arm Trapezoid Max Velocity", Constants.ArmConstants.trapezoidMaxVelocity), 
      SmartDashboard.getNumber("Arm Trapezoid Max Acceleration", Constants.ArmConstants.trapezoidMaxAcceleration));
    m_armPID.setConstraints(m_constraints);
  }

  // Controls a simple motor's position using a SimpleMotorFeedforward
  // and a ProfiledPIDController
  public void goToPosition(double goalPosition) {
    goal = goalPosition;
    m_armPID.setGoal(goalPosition);
    double pidVal = m_armPID.calculate(curAngle, goalPosition);
    double acceleration = (m_armPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    if(passedLimits()) m_arm.setVoltage(pidVal + m_armFeedforward.calculate(m_armPID.getSetpoint().velocity, acceleration));
    else m_arm.set(0);

    lastSpeed = m_armPID.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  public void stop() {
    m_arm.stopMotor();
  }

  public void setArmSpeed(double armSpeed) {
    if(passedLimits()) m_arm.set(0);
    else {
      if(armSpeed > 0.3) m_arm.set(Constants.ArmConstants.ArmSpeed);
      else if(armSpeed < -0.3) m_arm.set(-Constants.ArmConstants.ArmSpeed);
    }
  }

  public boolean passedLimits() {
    if(curAngle >= armTopLimit || curAngle <= armBotLimit) return true;
    else return false;
  }
}
