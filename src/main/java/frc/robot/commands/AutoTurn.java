// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurn extends CommandBase {

  private final double m_setpoint;

  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private boolean isFinished = false;

  private long time = -1; // -1 = invalid or no timestamp //
  private long startTime;

  /** Creates a new AutoTurn. */
  public AutoTurn(DrivetrainSubsystem drivetrain, double setpoint) {
    m_drivetrainSubsystem = drivetrain;
    m_setpoint = setpoint;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    System.out.println("Turn Command Started --------------------------------------------------");
    time = -1;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrainSubsystem.turnWithPID(m_setpoint);

    if (time < 0 && m_drivetrainSubsystem.isAtSetpoint()) {
      time = System.currentTimeMillis();
      System.out.println("Robot is at setpoint");
    } else if (time >= 0 && !m_drivetrainSubsystem.isAtSetpoint()) {
      time = -1;
      System.out.println("Robot is no longer at setpoint");
    }

    if (time > 0 && System.currentTimeMillis() - time >= 500) {
      isFinished = true;
      System.out.println("Command exited");
    }

    if (System.currentTimeMillis() - startTime > 3000) {
      isFinished = true;
      System.out.println("Command timed out");
    }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}