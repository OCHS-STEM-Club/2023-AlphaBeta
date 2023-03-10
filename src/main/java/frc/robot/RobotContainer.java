// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.AutoArmMove;
import frc.robot.commands.AutoDriveStraight;
import frc.robot.commands.Autos;
import frc.robot.commands.GrabberOn;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  public enum GamePieceMode {
    CUBE, CONE
  }

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final TankDriveCommand m_tankDriveCommand = new TankDriveCommand(m_drivetrainSubsystem);

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  private final HandSubsystem m_handSubsystem = new HandSubsystem();
  // private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // private final IntakeCommand
  // private final AutoDriveStraight m_autoDriveStraight = new
  // AutoDriveStraight(m_drivetrainSubsystem, m_distance, m_speed);
  // private final AutoArmMove m_armToMidAuto = new AutoArmMove(m_armSubsystem,
  // m_distance, m_speed);
  // private final GrabberOn m_grabberOn = new GrabberOn(m_handSubsystem,
  // m_speed);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static XboxController m_driverController;
  public static XboxController m_operatorController;
  public static XboxController m_buttonBox;

  public static GamePieceMode gamePieceMode = GamePieceMode.CUBE; // Start with cube //
  // public static XboxController m_driverController;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    m_tankDriveCommand.addRequirements(m_drivetrainSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(m_tankDriveCommand);

    // m_autoDriveStraight.addRequirements(m_drivetrainSubsystem);
    // SmartDashboard.putNumber("ArmEncoderValue",
    // m_armSubsystem.getArmEncoderDistance());
    // m_armCommand.addRequirements(m_armSubsystem);
    // m_armSubsystem.setDefaultCommand(m_armCommand);

    m_driverController = new XboxController(Constants.Operator.kdriverControllerPort);
    m_operatorController = new XboxController(Constants.Operator.koperatorControllerPort);
    m_buttonBox = new XboxController(Constants.Operator.kButtonBoxPort);

    SmartDashboard.putBoolean("GamePiece Mode", gamePieceMode == GamePieceMode.CONE); // Sets default mode to CUBE on
                                                                                      // SD//
    // m_driverController = new
    // XboxController(Constants.Operator.kdriverControllerPort);

    // if (m_driverController.getRightTriggerAxis() > 0.5) {
    // m_drivetrainSubsystem.setMaxOutput(1.5);
    // } else {
    // m_drivetrainSubsystem.setMaxOutput(1);
    // }

    if (m_driverController.getRawButton(7)) {
      m_drivetrainSubsystem.resetEncoders();
    }

    if (m_driverController.getRawButton(8)) {
      m_armSubsystem.resetArmEncoders();
    }

    configureBindings();

    // Shuffleboard.getTab("SmartDashboard").add(m_chooser);
    SmartDashboard.putData(m_chooser);

    m_chooser.setDefaultOption("Cube High Auto", Autos.exampleAuto(m_drivetrainSubsystem, m_armSubsystem, m_handSubsystem));
    m_chooser.addOption("Simple Auto", Autos.mobilityAuto(m_drivetrainSubsystem, m_handSubsystem));
    m_chooser.addOption("Mid Cone Auto", Autos.midConeMobilityAuto(m_drivetrainSubsystem, m_armSubsystem, m_handSubsystem));
    m_chooser.addOption("Drive Straight", Autos.driveStraight(m_drivetrainSubsystem));
    m_chooser.addOption("Cube High Station", Autos.highCubeCStation(m_drivetrainSubsystem, m_armSubsystem, m_handSubsystem));
    // Put the chooser on the dashboard
    
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

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new IntakeCommand(m_armSubsystem, m_handSubsystem));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new OuttakeCommand(m_armSubsystem, m_handSubsystem, 0.5));

    new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(new OuttakeCommand(m_armSubsystem, m_handSubsystem, 1));

    new JoystickButton(m_buttonBox, Button.kA.value)
        .onTrue(new InstantCommand(() -> {
          if (gamePieceMode == GamePieceMode.CONE) {
            gamePieceMode = GamePieceMode.CUBE;
            SmartDashboard.putBoolean("GamePiece Mode", false);
          } else if (gamePieceMode == GamePieceMode.CUBE) {
            gamePieceMode = GamePieceMode.CONE;
            SmartDashboard.putBoolean("GamePiece Mode", true);
          }
        }));

    

    // System.out.println("GamePieceMode Running");

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
  public Command getAutonomousCommand() {
    System.out.println("Auto is Running");
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void getUltrasonicSensorDistanceIn() {
    m_handSubsystem.getUltrasonicSensorDistanceIn();
  }

  public void setAutomaticModeUltrasonicSenor() {
    m_handSubsystem.setAutomaticModeUltrasonicSenor();
  }

  public void resetEncoders() {
    m_drivetrainSubsystem.resetEncoders();
  }

  public void stopHand() {
    m_handSubsystem.autoHandOn(0);
  }


}
