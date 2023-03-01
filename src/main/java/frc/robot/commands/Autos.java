// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;

public final class Autos {

  public static CommandBase driveStraight(DrivetrainSubsystem drivetrainSubsystem) {
    return Commands.sequence(
        new AutoDriveStraight(drivetrainSubsystem, -0.07, 0.5));
  }

  public static CommandBase mobilityAuto(DrivetrainSubsystem drivetrainSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
        new AutoDriveStraight(drivetrainSubsystem, 0.07, -0.5)
            .alongWith(new GrabberOn(handSubsystem, 0.5)));
  }

  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
    // return Commands.sequence(new AutoArmMove(armSubsystem));
    return Commands.sequence(
        // drive forward and move the arm to high, with the intake on
        new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
            .raceWith(new GrabberOn(handSubsystem, -0.5)),
        new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.35),
        new WaitCommand(2)
            .raceWith(new GrabberOn(handSubsystem, 0.35)),
        new AutoDriveStraight(drivetrainSubsystem, 0.08, -0.5),
        new AutoArmMove(armSubsystem, Constants.Setpoints.kresetSetpoint),
        new GrabberOn(handSubsystem, 0)
    );
  }

  public static CommandBase highCubeCStation(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, 
  HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
        .raceWith(new GrabberOn(handSubsystem, -0.5)),
      new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.35),
      new WaitCommand(2)
          .raceWith (new GrabberOn(handSubsystem, 0.35)),
      new AutoDriveStraight(drivetrainSubsystem, 0.04, -0.5),
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcarrySetpoint),
      new GrabberOn(handSubsystem, 0)
    );
  }
  
  public static CommandBase midConeMobilityAuto(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {

    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kconeMidSetpoint)
        .raceWith(new GrabberOn(handSubsystem, -0.5)),
      new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.35),
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeMidSetpoint),
      new WaitCommand(2)
        .raceWith(new GrabberOn(handSubsystem, 0)),
      new AutoArmMove(armSubsystem, Constants.Setpoints.kautoConeDropSetpoint),
      new AutoDriveStraight(drivetrainSubsystem, 0.01, -0.35),
      new GrabberOn(handSubsystem, 0)
    );

  }
    

  // new AutoDriveStraight(drivetrainSubsystem, 2, 0.5),
  // new AutoArmMove(armSubsystem, -900, 0.5),
  // new GrabberOn(handSubsystem, 0.5)

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
