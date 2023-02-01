// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.Autos;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final TankDriveCommand m_tankDriveCommand = new TankDriveCommand(m_drivetrainSubsystem);

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  //private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static XboxController m_driverController;
  public static XboxController m_operatorController;
 // public static XboxController m_driverController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_tankDriveCommand.addRequirements(m_drivetrainSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(m_tankDriveCommand);

    // m_armCommand.addRequirements(m_armSubsystem);
    // m_armSubsystem.setDefaultCommand(m_armCommand);

    m_driverController = new XboxController(Constants.Operator.kdriverControllerPort);
    m_operatorController = new XboxController(Constants.Operator.koperatorControllerPort);
    //m_driverController = new XboxController(Constants.Operator.kdriverControllerPort);

    // if (m_driverController.getLeftTriggerAxis() > 0.5) {
    //   m_drivetrainSubsystem.setMaxOutput(0.5);
    // } else {
    //   m_drivetrainSubsystem.setMaxOutput(0.5);
    // }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
