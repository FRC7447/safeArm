// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  double kDt = Constants.ArmConstants.trapezoidTimeFreq;

  CANSparkMax m_arm;
  SparkMaxPIDController m_armPID;
  SparkMaxAbsoluteEncoder m_armEncoder;

  ArmFeedforward m_armFeedforward;

  TrapezoidProfile.Constraints m_constraints;
  TrapezoidProfile.State setpoint;
  double rawGoal = 0.0;

  // FOR TESTING, DELETE AFTERWARDS
  double kP = 0.1; 
  double kI = 1e-4;
  double kD = 1; 
  double kIz = 0.5; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;
  /** Creates a new arm. */
  public Arm() {
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);
    m_arm.enableVoltageCompensation(12);
    m_arm.setInverted(false);

    m_armEncoder = m_arm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_armEncoder.setPositionConversionFactor(360);
    m_armEncoder.setVelocityConversionFactor(360);

    m_armPID = m_arm.getPIDController();
    m_armPID.setFeedbackDevice(m_armEncoder);

    m_armFeedforward = new ArmFeedforward(
      Constants.ArmConstants.armkS, Constants.ArmConstants.armkG,
      Constants.ArmConstants.armkV, Constants.ArmConstants.armkA);
    
    m_constraints = new TrapezoidProfile.Constraints(
      Constants.ArmConstants.trapezoidMaxVelocity, Constants.ArmConstants.trapezoidMaxAcceleration);

    // m_armPID.setSmartMotionMaxAccel(656.0, 0);
    // m_armPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // m_armPID.setSmartMotionMaxVelocity(1000, 0);

    m_arm.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_arm.setSoftLimit(SoftLimitDirection.kForward, Constants.ArmConstants.armUpperLimitf);
    m_arm.setSoftLimit(SoftLimitDirection.kReverse, Constants.ArmConstants.armLowerLimitf);

    m_armPID.setP(kP, 0);
    m_armPID.setI(kI, 0);
    m_armPID.setD(kD, 0);
    m_armPID.setIZone(kIz, 0);
    m_armPID.setOutputRange(kMinOutput, kMaxOutput, 0);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Degree", rawGoal);
    SmartDashboard.putBoolean("Forward Soft Limit Enabled", true);
    SmartDashboard.putNumber("Forward Soft Limit", 
      m_arm.getSoftLimit(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean("Reverse Soft Limit Enabled", true);
    SmartDashboard.putNumber("Reverse Soft Limit", 
      m_arm.getSoftLimit(SoftLimitDirection.kReverse));
    SmartDashboard.putNumber("Arm Trapezoid Max Velocity", Constants.ArmConstants.trapezoidMaxVelocity);
    SmartDashboard.putNumber("Arm Trapezoid Max Acceleration", Constants.ArmConstants.trapezoidMaxAcceleration);
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
    double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Voltage", m_arm.getBusVoltage());
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_armPID.setP(p); kP = p; }
    if((i != kI)) { m_armPID.setI(i); kI = i; }
    if((d != kD)) { m_armPID.setD(d); kD = d; }
    if((iz != kIz)) { m_armPID.setIZone(iz); kIz = iz; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_armPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    m_constraints = new TrapezoidProfile.Constraints(
      SmartDashboard.getNumber("Arm Trapezoid Max Velocity", Constants.ArmConstants.trapezoidMaxVelocity), 
      SmartDashboard.getNumber("Arm Trapezoid Max Acceleration", Constants.ArmConstants.trapezoidMaxAcceleration));
    rawGoal = SmartDashboard.getNumber("Set Rotations", rawGoal);
    if(rawGoal != 0.0) {
      TrapezoidProfile.State goalPos = new TrapezoidProfile.State(rawGoal, 0);
      TrapezoidProfile.State curPos = new TrapezoidProfile.State(m_armEncoder.getPosition(), 0);
      TrapezoidProfile profile = new TrapezoidProfile(m_constraints, goalPos, curPos);

      setpoint = profile.calculate(kDt);

      // This method will be called once per scheduler run
      m_armPID.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0, 
      m_armFeedforward.calculate(setpoint.position, setpoint.velocity));
    }
    
    SmartDashboard.putNumber("Arm SetPoint: ", rawGoal);
    SmartDashboard.putNumber("Arm Encoder Position", m_armEncoder.getPosition());
  }

  public void setGoal(double newGoal) {
    rawGoal = newGoal;
  }

  public void stop() {
    m_arm.stopMotor();
  }

  public void setArmSpeed(double armSpeed) {
    if(armSpeed > 0.3) m_arm.set(Constants.ArmConstants.ArmSpeed);
    else if(armSpeed < -0.3) m_arm.set(-Constants.ArmConstants.ArmSpeed);
  }
}