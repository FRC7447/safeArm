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
  public static class ArmJoystickConstants {
    public static final int joystickID = 0;

    public static final int gripXAxis = 0;
    public static final int gripYAxis = 1;
    public static final int gripTwistAxis = 2;
    public static final int throttleAxis = 3;

    public static final int buttonPOV0 = 0;
    public static final int buttonPOV180 = 180;

    public static final int button1 = 1;
    public static final int button2 = 2;
    public static final int button3 = 3;
    public static final int button4 = 4;
    public static final int button5 = 5;
    public static final int button6 = 6;
    public static final int button7 = 7;
    public static final int button8 = 8;
    public static final int button9 = 9;
    public static final int button10 = 10;
    public static final int button11 = 11;
    public static final int button12 = 12;
  }

  public static class WristConstants {
    // For Teleop
    public static final double WristSpeed = 0.1;

    // For auton commands
    // public static final double Pt3PositionWrist = 0.0;
    // public static final double Pt2PositionWrist = 0.0;
    // public static final double Pt1PositionWrist = 0.0;
    // public static final double TopPositionWrist = 0.0;
    // public static final double ShelfIntakePositionWrist = 0.0;

    public static final double wristUpperLimit = 0.0;
    public static final double wristLowerLimit = 0.0;

    public static final int wristID = 15;
    public static final int wristEncoderID = 0;
    // public static final double wristkP = 0.0;
    // public static final double wristkI = 0.0;
    // public static final double wristkD = 0.0;
  }

  public static class ArmConstants {
    // For Teleop
    public static final double ArmSpeed = 0.3;

    // For auton commands
    // public static final double Pt3PositionArm = 200.0;
    // public static final double Pt2PositionArm = 0.0;
    // public static final double Pt1PositionArm = 0.0;
    // public static final double RestPositionArm = 0.0;
    // public static final double HumanIntakePositionArm = 0.0; 
    // public static final double ShelfIntakePositionArm = 0.0;

    // ID ASSIGNMENT
    public static final int ArmID = 17;
    public static final int ArmEncoderID = 0;

    // TUNE FEEDFORWARD AND PID VALUES
    // public static final double armKP = 0.1; 
    // public static final double armKI = 0.0;
    // public static final double armKD = 0.0;

    // public static final double armkS = 0.0;
    // public static final double armkG = 0.0;
    // public static final double armkV = 0.0;
    // public static final double armkA = 0.0;

    // public static final double trapezoidMaxVelocity = 1.0;
    // public static final double trapezoidMaxAcceleration = 1.75;
    // public static final double trapezoidTimeFreq = 0.05; 

    public static final double armUpperLimit = 142.0;
    public static final double armLowerLimit = 240.0;

    // public static final float armUpperLimitf = 0.0f;
    // public static final float armLowerLimitf = 0.0f;
  }

  public static class TelescopeConstants {
    // For auton commands
    // public static final double fullyRetractedPosition = 0.0;
    // public static final double fullyExtendedPosition = 0.0;
    // public static final double groundIntakePosition = 0.0;

    public static final int telescopeID = 16;
    public static final double telescopeUpperLimit = -1045;
    public static final double telescopeLowerLimit = 0;
    public static final double telescopeSpeed = 0.3;

    // public static final double telescopeKP = 0.0; 
    // public static final double telescopeKI = 0.0;
    // public static final double telescopeKD = 0.0;
  }

  public static class ClawConstants {
    public static final int clawID = 14;

    public static final int analogID = 0;
    public static final int kUltrasonicPingPort = 0;
    public static final int kUltrasonicEchoPort = 0;

    public static final double acceptableDistanceFromObject = 0.0;
    public static final double clawSpeed = 0.0;
  }
}
