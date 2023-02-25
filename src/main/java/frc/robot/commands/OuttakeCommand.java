// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;

public class OuttakeCommand extends CommandBase {

  private final ArmSubsystem m_armSubsystem;
  private final HandSubsystem m_handSubsystem;
  private final double m_speed;

  /** Creates a new OuttakeCommand. */
  public OuttakeCommand(ArmSubsystem armSubsystem, HandSubsystem handSubsystem, double speed) {
    m_armSubsystem = armSubsystem;
    m_handSubsystem = handSubsystem;
    m_speed = speed;
    addRequirements(armSubsystem, handSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_handSubsystem.spinHandMotors(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_handSubsystem.spinHandMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
