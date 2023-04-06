// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.LimelightTracking;

public final class Autos {

  public static CommandBase mobilityAuto(DrivetrainSubsystem drivetrainSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
        new AutoDriveStraight(drivetrainSubsystem, 0.07, -0.25)
            .alongWith(new GrabberOn(handSubsystem, 0.5)));
  }

  /** Example static factory for an autonomous command. */
  public static CommandBase highCubeAuto(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
    // return Commands.sequence(new AutoArmMove(armSubsystem));
    return Commands.sequence(
        // drive forward and move the arm to high, with the intake on
        new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
            .raceWith(new GrabberOn(handSubsystem, -0.25)),
        new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.2),
        new WaitCommand(2)
            .raceWith(new GrabberOn(handSubsystem, 0.3)),
        new AutoDriveStraight(drivetrainSubsystem, 0.09, -0.35)
            .alongWith(new AutoArmMove(armSubsystem, Constants.Setpoints.kresetSetpoint))
            .raceWith(new GrabberOn(handSubsystem, 0))
    );
  }

  
  public static CommandBase midConeMobilityAuto(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {

    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kconeMidSetpoint)
        .raceWith(new GrabberOn(handSubsystem, -0.25)),
      new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.15),
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeMidSetpoint),
      new WaitCommand(2)
        .raceWith(new GrabberOn(handSubsystem, 0)),
      new AutoArmMove(armSubsystem, Constants.Setpoints.kautoConeDropSetpoint),
      new AutoDriveStraight(drivetrainSubsystem, 0.09, -0.35)
      .alongWith(new AutoArmMove(armSubsystem, Constants.Setpoints.kresetSetpoint))
      .raceWith(new GrabberOn(handSubsystem, 0))
    );
  }

  public static CommandBase autoTurn(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {

    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
        .raceWith(new GrabberOn(handSubsystem, -0.5)),
      new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.15),
      new WaitCommand(2)
        .raceWith(new GrabberOn(handSubsystem, 0.2)),
      new AutoDriveStraight(drivetrainSubsystem, 0.0005, -0.35),
      new WaitCommand(2),
      new AutoTurn(drivetrainSubsystem, -85),
      new AutoArmMove(armSubsystem, 0)
        .raceWith(new GrabberOn(handSubsystem, 0)),
      new AutoDriveStraight(drivetrainSubsystem, -0.04, 0.45),
      new AutoTurn(drivetrainSubsystem, -177),
      new AutoDriveStraight(drivetrainSubsystem, -0.07, 0.35)

    );
  }
    
  public static CommandBase autoTwoPieces(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
      .raceWith(new GrabberOn(handSubsystem, -0.5)),
    new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.25),
    new WaitCommand(2)
      .raceWith(new GrabberOn(handSubsystem, 0.2)),
    new AutoDriveStraight(drivetrainSubsystem, 0.07, -0.25)
      .raceWith(new AutoArmMove(armSubsystem, -10)),
    new WaitCommand(2),
    new AutoTurn(drivetrainSubsystem, -180)
    );
  }

  public static CommandBase autoChargeBalance(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
      .raceWith(new GrabberOn(handSubsystem, -0.5)),
    new AutoDriveStraight(drivetrainSubsystem, -0.01, 0.1),
    new WaitCommand(2)
      .raceWith(new GrabberOn(handSubsystem, 0.2)),
    new AutoDriveStraight(drivetrainSubsystem, 0.008, -0.15)
      .raceWith(new GrabberOn(handSubsystem, 0)),
    new WaitCommand(2)
      .raceWith(new AutoArmMove(armSubsystem, -10)),
    new AutoDriveStraight(drivetrainSubsystem, 0.035, -0.15)
      );
  }

  public static CommandBase autoBalanceWithMobility(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
            .raceWith(new GrabberOn(handSubsystem, -0.25)),
        new AutoDriveStraight(drivetrainSubsystem, -0.004, 0.3),
        new WaitCommand(1)
            .raceWith(new GrabberOn(handSubsystem, 0.3)),
      new AutoDriveStraight(drivetrainSubsystem, 0.02, -0.5)
        .raceWith(new GrabberOn(handSubsystem, 0))
        .raceWith(new AutoArmMove(armSubsystem, 55)),
      new AutoDriveStraight(drivetrainSubsystem, 0.05, -0.25),
      new AutoDriveStraight(drivetrainSubsystem, 0.04, -0.15),
      new AutoDriveStraight(drivetrainSubsystem, -0.04, 0.3)
        .andThen(new AutoBalance(drivetrainSubsystem, 0.1))
    );
  }

  public static CommandBase autoBalanceWithoutMobility(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
            .raceWith(new GrabberOn(handSubsystem, -0.25)),
        new AutoDriveStraight(drivetrainSubsystem, -0.005, 0.3),
        new WaitCommand(1)
            .raceWith(new GrabberOn(handSubsystem, 0.3)),
      new AutoDriveStraight(drivetrainSubsystem, 0.01, -0.2)
        .raceWith(new GrabberOn(handSubsystem, 0)),
      new AutoArmMove(armSubsystem, 55),
      new AutoDriveStraight(drivetrainSubsystem, 0.02, -0.2),
      new WaitCommand(1),
      new AutoDriveStraight(drivetrainSubsystem, 0.01, -0.2),
      new AutoBalance(drivetrainSubsystem, 0.1)
    );
  }

  public static CommandBase tossCubeAutoBalanceWithMobility(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
        .raceWith(new GrabberOn(handSubsystem, -0.25)),
      new AutoDriveStraight(drivetrainSubsystem, 0.03, -0.25)
        .raceWith(new GrabberOn(handSubsystem, 1)),
      new WaitCommand(1),
      new AutoDriveStraight(drivetrainSubsystem, 0.05, -0.25)
        .raceWith(new AutoArmMove(armSubsystem, 55))
        .raceWith(new GrabberOn(handSubsystem, 0)),
    new AutoDriveStraight(drivetrainSubsystem, 0.055, -0.15),
    new AutoDriveStraight(drivetrainSubsystem, -0.05, -0.15),
    new AutoBalance(drivetrainSubsystem, 0.09)
    );
  }

  public static CommandBase autoPIDStraight(DrivetrainSubsystem drivetrainSubsystem){
    return Commands.sequence(
      //new AutoTurn(drivetrainSubsystem, 90)
      new AutoPIDDrive(drivetrainSubsystem, 90, 0, 0)
    );
  }

  public static CommandBase autoCubeTrack(DrivetrainSubsystem drivetrainSubsystem, AprilTagTracking aprilTagTracking, ArmSubsystem armSubsystem, HandSubsystem handSubsystem, LimelightTracking limelightTracking) {
    return Commands.sequence(
    //  new AutoDriveStraight(drivetrainSubsystem, 0.05, -0.25)
    //   .raceWith(new CubeTracking(drivetrainSubsystem, aprilTagTracking, handSubsystem))
    new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeHighSetpoint)
      .raceWith(new GrabberOn(handSubsystem, -0.25)),
    new AutoDriveStraight(drivetrainSubsystem, -0.008, 0.2),
    new WaitCommand(0.5)
      .raceWith(new GrabberOn(handSubsystem, 0.25))
      .raceWith(new SetPipeline(aprilTagTracking)),
    new AutoDriveStraight(drivetrainSubsystem, 0.065, -0.5)
      .raceWith(new AutoArmMove(armSubsystem, -15))
      .raceWith(new GrabberOn(handSubsystem, 0)),
    new WaitCommand(0.5),
    new AutoTurn(drivetrainSubsystem, 175),
    new CubeTracking(drivetrainSubsystem, aprilTagTracking, handSubsystem)
      .raceWith( new GrabberOn(handSubsystem, -0.4)),
    new AutoTurn(drivetrainSubsystem, -10)
      .alongWith(new AutoArmMove(armSubsystem, 10)),
    new AutoDriveStraight(drivetrainSubsystem, -0.08, 0.4),
    new AutoAprilTagTracking(drivetrainSubsystem, limelightTracking, handSubsystem, -0.03)
      .alongWith(new AutoArmMove(armSubsystem, Constants.Setpoints.kcubeMidSetpoint)),
    new AutoDriveStraight(drivetrainSubsystem, 0.01, -0.2)
      .raceWith(new GrabberOn(handSubsystem, 0.25)) 

    // new CubeTracking(drivetrainSubsystem, aprilTagTracking, handSubsystem)
    // .alongWith(new GrabberOn(handSubsystem, -0.25))
    );
  }

  public static CommandBase turn(DrivetrainSubsystem drivetrainSubsystem, AprilTagTracking aprilTagTracking, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    return Commands.sequence(
      new AutoTurn(drivetrainSubsystem, -90),
      new GrabberOn(handSubsystem, 0.5)
    );
  }


  // new AutoDriveStraight(drivetrainSubsystem, 2, 0.5),
  // new AutoArmMove(armSubsystem, -900, 0.5),
  // new GrabberOn(handSubsystem, 0.5)
  public static CommandBase CubeTrack(DrivetrainSubsystem drivetrainSubsystem, AprilTagTracking aprilTagTracking, ArmSubsystem armSubsystem, HandSubsystem handSubsystem, LimelightTracking limelightTracking) {
    return Commands.sequence(
      new AutoAprilTagTracking(drivetrainSubsystem, limelightTracking, handSubsystem, 0.1)
    );
  }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
