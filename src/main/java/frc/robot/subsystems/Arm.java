// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax m_arm;
  DutyCycleEncoder m_armEncoder;
  ProfiledPIDController m_armPID;
  double goal = 0.0;
  ArmFeedforward m_armFeedforward;
  TrapezoidProfile.Constraints m_constraints;
  /** Creates a new arm. */
  public Arm() {
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);

    m_armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    m_armEncoder.setDistancePerRotation(8);
    m_armEncoder.get();

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

    m_armPID.setGoal(goal);
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  public void stop() {
    m_arm.stopMotor();
  }

  public void setArmSpeed(double armSpeed) {
    m_arm.set(armSpeed);
  }
}