// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveTrain {
    public static final int kleftDrive1Id = 5;
    public static final int kleftDrive2Id = 6;
    public static final int krightDrive1Id = 7;
    public static final int krightDrive2Id = 8;

    public static final double kspeedMultiplier = 0.85;

    // Never exceed 12 volts //
    public static final double kmaxDriveVoltage = 12;

    public static final double ksetOpenLoopRampRate = 0.95;

    // Drive Encoder Values //
    public static final double kdriveEncoderNativeUnitsPerRev = 42;
    public static final double kdriveGearRatio = 8.16;
    public static final double kwheelDiameterInches = 6; // TODO: Temporary wheel diameter

    public static final double kinchesToMeters = 0.0254;
  }

  public static class Operator {
    public static final int kdriverControllerPort = 0;
    public static final int koperatorControllerPort = 1;
    public static final int kButtonBoxPort = 3;
  }

  public static class Arm {
    public static final int karmMotor = 14;
    public static final double kspeedMultiplier = 0.5;
  }

  public static class Setpoints {
    public static final double kresetSetpoint = -10;
    public static final double kcarrySetpoint = 20;
    public static final double kconeMidSetpoint = 103;
    public static final double kcubeMidSetpoint = 80;
    public static final double kconeHighSetpoint = 125;
    public static final double kcubeHighSetpoint = 108;
    public static final double kautoConeDropSetpoint = 70;
    public static final double khumanPlayerStation = 95;
  }
  
  public static class LED {
    public static final int PWPORT = 0;
    public static final int BUFFERSIZE = 60;
  }
}
