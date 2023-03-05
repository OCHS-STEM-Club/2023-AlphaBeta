// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoArmMove extends CommandBase {
  private final double m_setpoint;

  private final ArmSubsystem m_armSubsystem;

  /** Creates a new ArmToMidAuto. */
  public AutoArmMove(ArmSubsystem arm, double setpoint) {
    m_armSubsystem = arm;
    m_setpoint = setpoint;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.goToPosition(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_setpoint >= 0) {
      return m_armSubsystem.getArmEncoderDistance() >= m_setpoint - 5;
    } else {
      return m_armSubsystem.getArmEncoderDistance() <= m_setpoint - 5;
    }
  }
}
