// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.AprilTagTracking;

public class LimelightAutoAlign extends CommandBase {
  /** Creates a new LimelightAutoAlign. */
  private final AprilTagTracking m_limelightTracking;
  private final DrivetrainSubsystem m_drivetrain;

  double visionmove;
  double visionturn;

  public LimelightAutoAlign(AprilTagTracking limelightTracking, DrivetrainSubsystem drivetrainSubsystem) {
    m_limelightTracking = limelightTracking;
    m_drivetrain = drivetrainSubsystem;
    addRequirements(limelightTracking, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_driverController.getXButton()) {
      visionturn = m_limelightTracking.trackTurn();
      m_drivetrain.setDrivetrainSpeed(visionturn, RobotContainer.m_driverController.getRawAxis(4) * 0.5);
    } 
    // else if (RobotContainer.m_driverController.getYButton()) {
    //   visionmove = m_limelightTracking.trackDrive();
    //   m_drivetrain.setDrivetrainSpeed(visionmove, RobotContainer.m_driverController.getRawAxis(1) * 0.5);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
