// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int rTrigger = 5;
  public static final int lTrigger = 6;
  public static final int rTab = 3;
  public static final int lTab = 4;

  public static class WristConstants {
    // For Teleop
    public static final double upWristSpeed = 0.1;
    public static final double downWristSpeed = -0.1;

    // For auton commands
    public static final double Pt3PositionWrist = 0.0;
    public static final double Pt2PositionWrist = 0.0;
    public static final double Pt1PositionWrist = 0.0;
    public static final double TopPositionWrist = 0.0;
    public static final double wristUpperLimit = 0.0;
    public static final double wristLowerLimit = 0.0;

    public static final int wristID = 0;
    public static final int intakeID = 0;
    public static final double wristkP = 0.0;
    public static final double wristkI = 0.0;
    public static final double wristkD = 0.0;
  }

  public static class ArmConstants {
    // For Teleop
    public static final double upArmSpeed = 0.1;
    public static final double downArmSpeed = -0.1;

    // For auton commands
    public static final double Pt3PositionArm = 0.0;
    public static final double Pt2PositionArm = 0.0;
    public static final double Pt1PositionArm = 0.0;
    public static final double RestPositionArm = 0.0;

    // ID ASSIGNMENT
    public static final int ArmID = 0;
    public static final int ArmEncoderID = 0;

    // TUNE FEEDFORWARD AND PID VALUES
    public static final double armKP = 0.0; 
    public static final double armKI = 0.0;
    public static final double armKD = 0.0;

    public static final double armFF = 0.0;

    public static final float armUpperLimit = 0.0f;
    public static final float armLowerLimit = 0.0f;
  }

  public static class TelescopeConstants {
    // For auton commands
    public static final double fullyRetractedPosition = 0.0;
    public static final double fullyExtendedPosition = 0.0;
    public static final double groundIntakePosition = 0.0;

    public static final int telescopeID = 0;
    public static final double telescopeUpperLimit = 0.0;
    public static final double telescopeLowerLimit = 0.0;
    public static final double telescopeSpeed = 0.1;

    public static final double telescopeKP = 0.0; 
    public static final double telescopeKI = 0.0;
    public static final double telescopeKD = 0.0;
  }
}
