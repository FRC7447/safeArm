// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase {
  CANSparkMax m_claw;
  AnalogPotentiometer AnalogDistanceSensor;
  Ultrasonic UltraDistanceSensor;
  /** Creates a new Claw. */
  public Claw() {
    m_claw = new CANSparkMax(Constants.ClawConstants.clawID, MotorType.kBrushless);
    AnalogDistanceSensor = new AnalogPotentiometer(Constants.ClawConstants.analogID, 100, 0);
    UltraDistanceSensor = new Ultrasonic(
      Constants.ClawConstants.kUltrasonicPingPort, 
      Constants.ClawConstants.kUltrasonicEchoPort);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ultra in Inches: ", UltraDistanceSensor.getRangeInches());
    SmartDashboard.putNumber("Ultra in Milimeters: ", UltraDistanceSensor.getRangeMM());
    SmartDashboard.putNumber("Analog Distance: ", AnalogDistanceSensor.get());
    // This method will be called once per scheduler run
  }

  public void setClawSpeed(double clawSpeed) {
    m_claw.set(clawSpeed);
  }

  public double getDistanceFromObject() {
    UltraDistanceSensor.getRangeMM();
    UltraDistanceSensor.getRangeInches();

    return AnalogDistanceSensor.get();
  }
}
