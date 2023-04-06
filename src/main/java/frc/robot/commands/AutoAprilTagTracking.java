// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.LimelightTracking;

public class AutoAprilTagTracking extends CommandBase {

  private final HandSubsystem m_handSubsystem;
  private final LimelightTracking m_limeLightTagTracking;
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  double visionmove;
  double visionturn;
  double m_distance;
  /** Creates a new AutoAprilTagTracking. */
  public AutoAprilTagTracking(DrivetrainSubsystem drivetrain, LimelightTracking tracking, HandSubsystem hand, double distance) {
    m_drivetrainSubsystem = drivetrain;
    m_limeLightTagTracking = tracking;
    m_handSubsystem = hand;
    m_distance = distance;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLightTagTracking.pipelineSet1();
    m_drivetrainSubsystem.setEncodersToZero();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionmove = m_limeLightTagTracking.trackDrive();
    visionturn = m_limeLightTagTracking.trackTurn();
    m_drivetrainSubsystem.setDrivetrainSpeed(visionmove, visionturn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drivetrainSubsystem.getLeftEncoderDistance() < m_distance) {
      return m_drivetrainSubsystem.getLeftEncoderDistance() < m_distance;
    } else return false;
  }
}
