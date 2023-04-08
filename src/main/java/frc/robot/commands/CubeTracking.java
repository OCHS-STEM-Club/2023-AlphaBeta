// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;

public class CubeTracking extends CommandBase {
  double visionmove;
  double visionturn;
  
  private final HandSubsystem m_handSubsystem;
  private final AprilTagTracking m_aprilTagTracking;
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  double m_distance;
  /** Creates a new CubeTracking. */
  public CubeTracking(DrivetrainSubsystem drivetrain, AprilTagTracking tracking, HandSubsystem hand, double distance) {
    m_drivetrainSubsystem = drivetrain;
    m_aprilTagTracking = tracking;
    m_handSubsystem = hand;
    m_distance = distance;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setEncodersToZero();
    m_aprilTagTracking.pipelineSet0();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionmove = m_aprilTagTracking.trackDrive();
    visionturn = m_aprilTagTracking.trackTurn();
    m_drivetrainSubsystem.setDrivetrainSpeed(visionmove, visionturn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_aprilTagTracking.pipelineSet1();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (m_drivetrainSubsystem.getLeftEncoderDistance() < m_distance) {
        return m_drivetrainSubsystem.getLeftEncoderDistance() < m_distance;
      } else {
        return m_handSubsystem.gamePieceInHand() <= 3;
      }
  }
}
