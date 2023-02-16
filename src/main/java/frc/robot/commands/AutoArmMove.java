// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoArmMove extends CommandBase {
  private final double m_distance;
  private final double m_speed;

  private final ArmSubsystem m_armSubsystem;
  /** Creates a new ArmToMidAuto. */
  public AutoArmMove(ArmSubsystem arm, double distance, double speed) {
    m_armSubsystem = arm;
    m_distance = distance;
    m_speed = speed;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.resetArmEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.armUpAuto(m_speed, m_distance);
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
