// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.Autos;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.ArmPIDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainPIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  //private final DrivetrainPIDSubsystem m_drivetrainPIDSubsystem = new DrivetrainPIDSubsystem();
  
  //private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ArmPIDSubsystem m_armSubsystem = new ArmPIDSubsystem();

  private final HandSubsystem m_handSubsystem = new HandSubsystem();
  //private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
 
 

  private final Command m_armTest = Commands.runOnce(m_armSubsystem::enable, m_armSubsystem);

  //private final Command m_autoDrive = Commands.runOnce(m_drivetrainPIDSubsystem::enable, m_drivetrainPIDSubsystem);
 // public static XboxController m_driverController;

  public static CommandXboxController m_driverController = new CommandXboxController(Constants.Operator.kdriverControllerPort);
  public static CommandXboxController m_operatorController = new CommandXboxController(Constants.Operator.koperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_tankDriveCommand.addRequirements(m_drivetrainSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(m_tankDriveCommand);

    //SmartDashboard.putNumber("ArmEncoderValue", m_armSubsystem.getArmEncoderDistance());
    // m_armCommand.addRequirements(m_armSubsystem);
  // m_armSubsystem.setDefaultCommand(m_armCommand);

    
    //m_driverController = new XboxController(Constants.Operator.kdriverControllerPort);


    // if (m_driverController.getRightTriggerAxis() > 0.5) {
    //   m_drivetrainSubsystem.setMaxOutput(1.5);
    // } else {
    //   m_drivetrainSubsystem.setMaxOutput(1);
    // }

    if (m_operatorController.a().getAsBoolean() == true) {
      m_armSubsystem.setSetpoint(-911); 
    }

    // if (m_driverController.y().getAsBoolean() == true) {
    //   new DrivePIDCommand(90, m_drivetrainSubsystem).withTimeout(5);
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

    //m_operatorController.a().onTrue(m_armTest);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }




  Command m_autoArmtoMid =
     Commands.sequence(
        Commands.runOnce(m_armSubsystem::enable, m_armSubsystem),

        Commands.waitUntil(m_armSubsystem::atSetPoint))

      .withTimeout(5)

      .andThen(
        Commands.runOnce(
            () -> {
              m_armSubsystem.disable();
            }));

  

  // Command m_autonomousCommand =
  //     Command.sequence(
  //       Command.runOnce(m_drivetrainPIDSubsystem::enable, m_drivetrainPIDSubsystem),

  //       Command.waitUntil()
  //     )


  // public void getEncoderValues() {
  //   m_drivetrainSubsystem.printEncoders();
  // }

  // public void getArmEncoderValues() {
  //   m_armSubsystem.getArmEncoderValues();
  // }

  public void getUltrasonicSensorDistanceIn() {
    m_handSubsystem.getUltrasonicSensorDistanceIn();
  }

  public void setAutomaticModeUltrasonicSenor(){
    m_handSubsystem.setAutomaticModeUltrasonicSenor();
  }

  public void encoderValue() {
    m_armSubsystem.encoderValue();
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // An example command will be run in autonomous
    return m_armTest;
  }

}
