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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  CANSparkMax m_arm;
  GenericHID armJoystick;

  DutyCycleEncoder m_armEncoder;
  ProfiledPIDController m_armPID;

  boolean wedonotwantogoupanymore = false;
  boolean wedonotwantogodownanymore = false;

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
    armJoystick = new GenericHID(Constants.ArmJoystickConstants.joystickID);

    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    m_arm.setIdleMode(IdleMode.kBrake);
    m_arm.enableVoltageCompensation(12);
    m_arm.setInverted(false);

    m_armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    m_armEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    
    armTopLimit = Constants.ArmConstants.armUpperLimit;
    armBotLimit = Constants.ArmConstants.armLowerLimit;
    SmartDashboard.putNumber("Arm Top Limit", Constants.ArmConstants.armUpperLimit);
    SmartDashboard.putNumber("Arm Bot Limit", Constants.ArmConstants.armLowerLimit);
  }

  @Override
  public void periodic() {
    this.setArmSpeed(armJoystick.getRawAxis(Constants.ArmJoystickConstants.gripYAxis));
    curAngle = m_armEncoder.getAbsolutePosition() * 360;

    passedLimits();

    SmartDashboard.putNumber("Arm Goal Setpoint: ", goal);
    SmartDashboard.putNumber("Arm Current Absolute Angle: ", curAngle);
    SmartDashboard.putNumber("Arm Applied Output", m_arm.getAppliedOutput());
    SmartDashboard.putNumber("Arm Encoder Offset: ", m_armEncoder.getPositionOffset());

    // FOR VALE TUNING
    armTopLimit = SmartDashboard.getNumber("Arm Top Limit", Constants.ArmConstants.armUpperLimit);
    armBotLimit = SmartDashboard.getNumber("Arm Bot Limit", Constants.ArmConstants.armLowerLimit);
  }

  public void stop() {
    m_arm.stopMotor();
  }

  public void setArmSpeed(double armSpeed) {
    if( armSpeed > 0.2 ) {
      if(wedonotwantogodownanymore) {
        if( armJoystick.getRawAxis(Constants.ArmJoystickConstants.gripYAxis) < 0.0 ) {
          m_arm.set(Constants.ArmConstants.ArmSpeed);
        } else {
          m_arm.set(0);
        }
      } else {
        m_arm.set(-Constants.ArmConstants.ArmSpeed);
      } 
    } else if( armSpeed < -0.2 ) {
      if (wedonotwantogoupanymore) {
        if( armJoystick.getRawAxis(Constants.ArmJoystickConstants.gripYAxis) > 0.0 ) {
          m_arm.set(-Constants.ArmConstants.ArmSpeed);
        } else {
          m_arm.set(0);
        }
      } else {
        m_arm.set(Constants.ArmConstants.ArmSpeed);
      }
    } else {
      m_arm.set(0);
    }
  }

  public void passedLimits() {
    if( curAngle <= armTopLimit ) {
      wedonotwantogoupanymore = true;
      wedonotwantogodownanymore = false;
    } else if( curAngle >= armBotLimit ) {
      wedonotwantogodownanymore = true;
      wedonotwantogoupanymore = false;
    } else if( curAngle <= armBotLimit && curAngle >= armTopLimit ) {
      wedonotwantogodownanymore = false;
      wedonotwantogoupanymore = false;
    }
  }
}
