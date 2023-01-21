// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final XboxController m_driverController = new XboxController(
      Constants.Operator.kdriverControllerPort);
  public static final XboxController m_operatorController = new XboxController(
      Constants.Operator.kdriverControllerPort);

  public final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  // public static final XboxController m_driverController = new XboxController(
  //     Constants.Operator.kdriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value); //TODO: CHANGE//
    if (m_driverController.getLeftTriggerAxis() > 0.5) {
      m_drivetrainSubsystem.setMaxOutput(1);
    } else {
      m_drivetrainSubsystem.setMaxOutput(0.5);
    }
      // .onTrue(new InstantCommand(() -> m_drivetrainSubsystem.setMaxOutput(1)))
      // .onFalse(new InstantCommand(() -> m_drivetrainSubsystem.setMaxOutput(0.5)));
      

    m_tankDriveCommand.addRequirements(m_drivetrainSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(m_tankDriveCommand);
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
    
    m_LEDSubsystem.setBallLEDs(m_operatorController);

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
    // Create a voltage constraint to ensure we don't accelerate too fast in auto 
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.Trajectory.ksVolts, 
        Constants.Trajectory.kvVoltSecondsPerMeter,
        Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
      Constants.Trajectory.kDriveKinematics,
      10
    );

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            Constants.Trajectory.kMaxSpeedMetersPerSecond,
            Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Trajectory.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory =
     TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0,new Rotation2d(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, new Rotation2d(0)), 
      config);

    RamseteCommand ramsetecommand = 
    new RamseteCommand(
      exampleTrajectory,
      m_drivetrainSubsystem::getPose,
      new RamseteController(Constants.Trajectory.kRamseteB, Constants.Trajectory.kRamseteZeta),
      new SimpleMotorFeedforward(
        Constants.Trajectory.ksVolts,
        Constants.Trajectory.kvVoltSecondsPerMeter,
        Constants.Trajectory.kaVoltSecondsSquaredPerMeter),
      Constants.Trajectory.kDriveKinematics,
      m_drivetrainSubsystem::getWheelSpeeds,
      new PIDController(Constants.Trajectory.kPDriveVel, 0, 0),
      new PIDController(Constants.Trajectory.kPDriveVel, 0, 0),
      m_drivetrainSubsystem::tankDriveVolts,
      m_drivetrainSubsystem
      );
    
    m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return ramsetecommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0)); 
  }
}
