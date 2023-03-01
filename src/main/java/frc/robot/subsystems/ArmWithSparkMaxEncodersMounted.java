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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmWithSparkMaxEncodersMounted extends SubsystemBase {
  CANSparkMax m_arm;
  SparkMaxPIDController m_armPID;
  SparkMaxAbsoluteEncoder m_armEncoder;
  SparkMaxLimitSwitch m_forwardLimit;
  SparkMaxLimitSwitch m_reverseLimit;

  double goal = 0.0;

  // FOR TESTING, DELETE AFTERWARDS
  double kP = 0.1; 
  double kI = 1e-4;
  double kD = 1; 
  double kIz = 0; 
  double kFF = 0; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;
  /** Creates a new arm. */
  public ArmWithSparkMaxEncodersMounted() {
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);
    m_armEncoder = m_arm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_armEncoder.setPositionConversionFactor(360);
    m_armEncoder.setVelocityConversionFactor(360);
    goal = m_armEncoder.getPosition();
    m_armPID = m_arm.getPIDController();
    m_armPID.setFeedbackDevice(m_armEncoder);

    m_armPID.setSmartMotionMaxAccel(656.0, 0);
    m_armPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_armPID.setSmartMotionMaxVelocity(1000, 0);

    m_arm.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_arm.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmConstants.armUpperLimit);
    // m_arm.setSoftLimit(SoftLimitDirection.kReverse, Constants.ArmConstants.armLowerLimit);

    m_armPID.setP(kP);
    m_armPID.setI(kI);
    m_armPID.setD(kD);
    m_armPID.setIZone(kIz);
    m_armPID.setFF(kFF);
    m_armPID.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", goal);
  }

  @Override
  public void periodic() {
    m_arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 
    SmartDashboard.getBoolean("Forward Soft Limit Enabled", true));
    m_arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 
    SmartDashboard.getBoolean("Reverse Soft Limit Enabled", true));
    m_arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 
      (float)SmartDashboard.getNumber("Forward Soft Limit", Constants.ArmConstants.armUpperLimit));
    m_arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
      (float)SmartDashboard.getNumber("Reverse Soft Limit", Constants.ArmConstants.armLowerLimit));
    
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", kP);
    double i = SmartDashboard.getNumber("I Gain", kI);
    double d = SmartDashboard.getNumber("D Gain", kD);
    double iz = SmartDashboard.getNumber("I Zone", kIz);
    double ff = SmartDashboard.getNumber("Feed Forward", kFF);
    double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Voltage", m_arm.getBusVoltage());
    goal = SmartDashboard.getNumber("Set Rotations", goal);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_armPID.setP(p); kP = p; }
    if((i != kI)) { m_armPID.setI(i); kI = i; }
    if((d != kD)) { m_armPID.setD(d); kD = d; }
    if((iz != kIz)) { m_armPID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_armPID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_armPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // This method will be called once per scheduler run
    m_armPID.setReference(goal, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("Arm SetPoint: ", goal);
    SmartDashboard.putNumber("Arm Encoder Position", m_armEncoder.getPosition());
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