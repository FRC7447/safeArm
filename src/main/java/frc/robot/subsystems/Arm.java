// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  PIDController armPID;
  CANSparkMax m_arm;
  DutyCycleEncoder armEncoder;
  ArmFeedforward armFeedforward;
  
  double encoderPos = 0.0;
  double encoderRate = 0.0;
  double curEncoderPos = 0.0;

  double goal = 0.0;
  double output = 0.0;
  /** Creates a new arm. */
  public Arm() {
    m_arm = new CANSparkMax(Constants.ArmConstants.ArmID, MotorType.kBrushless);
    armPID = new PIDController(
      Constants.ArmConstants.armKP, 
      Constants.ArmConstants.armKI, 
      Constants.ArmConstants.armKD
    );
    armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoderID);
    armFeedforward = new ArmFeedforward(
      Constants.ArmConstants.armkS, 
      Constants.ArmConstants.armkG, 
      Constants.ArmConstants.armkV
    );
  }

  @Override
  public void periodic() {
    curEncoderPos = armEncoder.getAbsolutePosition();
    // encoderRate = (curEncoderPos - encoderPos) / 60;
    encoderPos = curEncoderPos;

    double pidOut = armPID.calculate(armEncoder.getAbsolutePosition(), goal);
    double feedForwardOut = 0.0; // armFeedforward.calculate(armEncoder.getDistance(), encoderRate);
    output = pidOut + feedForwardOut;

    if (encoderPos > Constants.ArmConstants.armUpperLimit) {
      output = 0.0;
      stop();
    } else if (encoderPos < Constants.ArmConstants.armLowerLimit) {
      output = 0.0;
      stop();
    }

    m_arm.set(output);

    if(armEncoder.getAbsolutePosition() == goal) {
      output = 0.0;
      stop();
    }
    // This method will be called once per scheduler run
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  public void stop() {
    m_arm.stopMotor();
  }

  public boolean isMoving() {
    return output != 0.0;
  }
}
