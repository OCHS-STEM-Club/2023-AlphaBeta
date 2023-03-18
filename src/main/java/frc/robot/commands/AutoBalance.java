// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {

  private final double m_speed;

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrain, double speed) {
    m_drivetrainSubsystem = drivetrain;
    m_speed = speed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drivetrainSubsystem.getRollAngle() < -8.5) {
      m_drivetrainSubsystem.setDrivetrainSpeed((-m_speed), 0);
    } else if (m_drivetrainSubsystem.getRollAngle() >= -8.5 && m_drivetrainSubsystem.getRollAngle() <= 8.5) {
      m_drivetrainSubsystem.setDrivetrainSpeed(0, 0);
    } else if (m_drivetrainSubsystem.getRollAngle() > 8.5) {
      m_drivetrainSubsystem.setDrivetrainSpeed((m_speed), 0);
    }
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
