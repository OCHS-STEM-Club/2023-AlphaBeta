// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveStraight extends CommandBase {

  private final double m_distance;
  private final double m_speed;

  private final DrivetrainSubsystem m_drivetrainSubsystem;

  /** Creates a new AutoDriveStraight. */
  public AutoDriveStraight(DrivetrainSubsystem drivetrain, double distance, double speed) {
    m_drivetrainSubsystem = drivetrain;
    m_distance = distance;
    m_speed = speed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setEncodersToZero();
    System.out.println(m_drivetrainSubsystem.getSpeed());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.setDrivetrainSpeed(Math.abs(m_speed) * Math.signum(m_distance), 0);
    // m_drivetrainSubsystem.auto(m_distance, m_speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrainSubsystem.auto(m_distance, m_speed);
    m_drivetrainSubsystem.setDrivetrainSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_distance >= 0) {
      return m_drivetrainSubsystem.getLeftEncoderDistance() >= m_distance;
    } else {
      return m_drivetrainSubsystem.getLeftEncoderDistance() <= m_distance;
    }
  }
}
