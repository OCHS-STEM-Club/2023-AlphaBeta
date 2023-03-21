// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AprilTagTracking extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-kone");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tl = table.getEntry("tl");

  NetworkTableEntry tid = table.getEntry("tid");

  NetworkTableEntry pl = table.getEntry("pipeline");


  double targetOffsetAngle_Vertical = ty.getDouble(0.0);
  double targetOffsetAngle_Horizontal = tx.getDouble(0.0);
  double targetArea = ta.getDouble(0.0);
  double targetSkew = tl.getDouble(0.0);
  double id = tid.getInteger(0);
  double pipeline = pl.getDouble(0);

  private double targetValue;
  private double turnOutput;
  private double driveOutput;
  private final double MAX_STEER = 0.065;
  private final double STEER_K = 0.025;

  

  public AprilTagTracking() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("tl");

    NetworkTableEntry tid = table.getEntry("tid");

  

    //System.out.println(trackTurn());
   

  }

  private double clamp(double in, double minval, double maxval) {
    if (in > maxval) {
      return maxval;
    }
    else if (in < minval) {
      return minval;
    }
    else {
      //System.out.print("else");
      return in;
    }
  }

  public double trackTurn() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-kone");
    NetworkTableInstance.getDefault().getTable("limelight-kone").getEntry("pipeline").setNumber(0);
    targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0.0);
    targetValue = table.getEntry("tv").getDouble(0.0);
    SmartDashboard.putNumber("tv", targetValue);
    

    if (targetValue == 1) {
      turnOutput = targetOffsetAngle_Horizontal* STEER_K; //* STEER_K
      //System.out.println(turnOutput);
      turnOutput = clamp(turnOutput, -MAX_STEER, MAX_STEER);
      //System.out.println(turnOutput);
      return turnOutput;
    } else return 0;
  }

  public double trackDrive() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-kone");
    NetworkTableInstance.getDefault().getTable("limelight-kone").getEntry("pipeline").setNumber(0);
    targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0);
    targetValue = table.getEntry("tv").getDouble(0.0);
    SmartDashboard.putNumber("tv", targetValue);

    if (targetValue == 1) {
      driveOutput = targetOffsetAngle_Vertical * STEER_K;
      driveOutput = clamp(driveOutput, -0.25, 0.25);
      return driveOutput;
    } else return 0;
  }

  
}
