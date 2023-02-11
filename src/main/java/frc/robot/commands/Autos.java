// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem ) {
    //return Commands.sequence(new ArmToMidAuto(armSubsystem));
    return Commands.parallel(
      new GrabberOn(handSubsystem, -0.5), 
      new AutoArmMove(armSubsystem, -900, 0.5),
      new AutoDriveStraight(drivetrainSubsystem, 1, -0.5)
      );
  }

  // new AutoDriveStraight(drivetrainSubsystem, 2, 0.5),
  //                            new AutoArmMove(armSubsystem, -900, 0.5), 
  //                            new GrabberOn(handSubsystem, 0.5)

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
