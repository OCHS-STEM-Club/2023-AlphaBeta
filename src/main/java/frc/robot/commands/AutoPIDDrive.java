// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoPIDDrive extends CommandBase {
  private final double m_setpoint;

  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private boolean isFinished = false;

  private final double m_distance;
  private final double m_speed;

  private long time = -1; // -1 = invalid or no timestamp //
  private long startTime;

  /** Creates a new AutoPIDDrive. */
  public AutoPIDDrive(DrivetrainSubsystem drivetrain, double setpoint, double distance, double speed) {
    m_drivetrainSubsystem = drivetrain;
    m_setpoint = setpoint;
    m_distance = distance;
    m_speed = speed;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.setEncodersToZero();

    isFinished = false;
    System.out.println("Turn Command Started --------------------------------------------------");
    time = -1;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.turnWithPID(m_setpoint);
    //m_drivetrainSubsystem.setDrivetrainSpeed(Math.abs(m_speed) * Math.signum(m_distance), 0);

  //   if (time < 0 && m_drivetrainSubsystem.isAtSetpoint()) {
  //     time = System.currentTimeMillis();
  //     System.out.println("Robot is at setpoint");
  //   } else if (time >= 0 && !m_drivetrainSubsystem.isAtSetpoint()) {
  //     time = -1;
  //     System.out.println("Robot is no longer at setpoint");
  //   }

  //   if (time > 0 && System.currentTimeMillis() - time >= 500) {
  //     isFinished = true;
  //     System.out.println("Command exited");
  //   }

  //   if (System.currentTimeMillis() - startTime > 3000) {
  //     isFinished = true;
  //     System.out.println("Command timed out");
  //   }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return isFinished;
   return false;
  }
}
