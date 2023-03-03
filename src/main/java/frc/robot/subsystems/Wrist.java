// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  CANSparkMax m_Wrist;
  DutyCycleEncoder m_WristEncoder;
  PIDController m_WristPID;
  double goal; 
  double curAngle;
  double WristTopLimit;
  double WristBotLimit;
  /** Creates a new Wrist. */
  public Wrist() {
    m_Wrist = new CANSparkMax(Constants.WristConstants.wristID, MotorType.kBrushless);
    m_Wrist.setIdleMode(IdleMode.kBrake);
    m_Wrist.enableVoltageCompensation(12);
    m_Wrist.setInverted(false);

    m_WristEncoder = new DutyCycleEncoder(Constants.WristConstants.wristEncoderID);
    m_WristEncoder.setDutyCycleRange(1 / 1025, 1024 / 1025);
    
    m_WristPID = new PIDController(
      Constants.WristConstants.wristkP, 
      Constants.WristConstants.wristkI, 
      Constants.WristConstants.wristkD);
    
    WristTopLimit = Constants.WristConstants.wristUpperLimit;
    WristBotLimit = Constants.WristConstants.wristLowerLimit;
    SmartDashboard.putNumber("Wrist Top Limit", Constants.WristConstants.wristUpperLimit);
    SmartDashboard.putNumber("Wrist Bot Limit", Constants.WristConstants.wristLowerLimit);
    // FOR TUNING
    SmartDashboard.putNumber("Wrist P Gain", Constants.WristConstants.wristkP);
    SmartDashboard.putNumber("Wrist I Gain", Constants.WristConstants.wristkI);
    SmartDashboard.putNumber("Wrist D Gain", Constants.WristConstants.wristkD);
  }

  @Override
  public void periodic() {
    curAngle = m_WristEncoder.getAbsolutePosition() * 360;

    SmartDashboard.putNumber("Wrist Goal Setpoint: ", goal);
    SmartDashboard.putNumber("Wrist Current Absolute Angle: ", curAngle);
    SmartDashboard.putNumber("Wrist Applied Output", m_Wrist.getAppliedOutput());
    SmartDashboard.putNumber("Wrist Encoder Offset: ", m_WristEncoder.getPositionOffset());

    // FOR VALE TUNING
    WristTopLimit = SmartDashboard.getNumber("Wrist Top Limit", Constants.WristConstants.wristUpperLimit);
    WristBotLimit = SmartDashboard.getNumber("Wrist Bot Limit", Constants.WristConstants.wristLowerLimit);

    m_WristPID.setP(SmartDashboard.getNumber("Wrist P Gain", Constants.WristConstants.wristkP));
    m_WristPID.setI(SmartDashboard.getNumber("Wrist I Gain", Constants.WristConstants.wristkI));
    m_WristPID.setD(SmartDashboard.getNumber("Wrist D Gain", Constants.WristConstants.wristkD));
  }

  // Controls a simple motor's position using a SimpleMotorFeedforward
  // and a ProfiledPIDController
  public void goToPosition(double goalPosition) {
    goal = goalPosition;
    double pidVal = m_WristPID.calculate(curAngle, goalPosition);

    if(passedLimits()) m_Wrist.setVoltage(pidVal);
    else m_Wrist.set(0);
  }

  public void stop() {
    m_Wrist.stopMotor();
  }

  public void setWristSpeed(double WristSpeed) {
    if(passedLimits()) m_Wrist.set(0);
    else {
      if(WristSpeed > 0.3) m_Wrist.set(Constants.WristConstants.WristSpeed);
      else if(WristSpeed < -0.3) m_Wrist.set(-Constants.WristConstants.WristSpeed);
    }
  }

  public boolean passedLimits() {
    if(curAngle >= WristTopLimit || curAngle <= WristBotLimit) return true;
    else return false;
  }
}