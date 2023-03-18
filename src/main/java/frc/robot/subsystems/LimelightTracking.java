// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Key;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightTracking extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-kone3");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tl = table.getEntry("tl");

  double targetOffsetAngle_Vertical = ty.getDouble(0.0);
  double targetOffsetAngle_Horizontal = tx.getDouble(0.0);
  double targetArea = ta.getDouble(0.0);
  double targetSkew = tl.getDouble(0.0);


  private double targetValue;
  private double turnOutput;
  private double driveOutput;
  private final double  MAX_STEER = 0.065;
  private final double STEER_K = 0.025;
  /** Creates a new LimelightTracking. */

  public LimelightTracking() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("tl");
  }

  public double trackTurn() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-kone3");
    targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0.0);
    targetValue = table.getEntry("tv").getDouble(0.0);
    SmartDashboard.putNumber("tv", targetValue);

    if (targetValue == 1) {
      driveOutput = targetOffsetAngle_Vertical * STEER_K;
    //  driveOutput = clamp(driveOutput -MAX_STEER, MAX_STEER);
    return 0;
    }
    return 0;
  }
}
