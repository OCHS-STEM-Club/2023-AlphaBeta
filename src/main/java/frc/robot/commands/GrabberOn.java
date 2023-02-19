// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;

public class GrabberOn extends CommandBase {

  private final HandSubsystem m_handSubsystem;
  private final double m_speed;

  /** Creates a new FeederOn. */
  public GrabberOn(HandSubsystem hand, double speed) {
    m_handSubsystem = hand;
    m_speed = speed;
    addRequirements(hand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_handSubsystem.spinHandMotors(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
