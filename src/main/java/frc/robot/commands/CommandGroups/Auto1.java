// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoArmMove;
import frc.robot.commands.GrabberOn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {

  /** Creates a new Auto1. */
  public Auto1(
    DrivetrainSubsystem m_drivetrainSubsystem,
    ArmSubsystem m_armSubsystem,
    HandSubsystem m_handSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_handSubsystem.autoHandOn(0.5))
        .alongWith(
          new WaitCommand(3)
            .andThen(new InstantCommand(() -> m_armSubsystem.armUpAuto(0.5, -900)))));
          new WaitCommand(3)
            .andThen(new InstantCommand(() -> m_armSubsystem.armMotorSet(0)));
        
  }
}
