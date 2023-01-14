// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private MotorControllerGroup leftDrive;
  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  private MotorControllerGroup rightDrive;


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftDrive1 = new CANSparkMax(Constants.kleftDrive1, MotorType.kBrushless);
      leftDrive1.setInverted(true);
    leftDrive2 = new CANSparkMax(Constants.kleftDrive2, MotorType.kBrushless);
      leftDrive2.setInverted(true);
    rightDrive1 = new CANSparkMax(Constants.krightDrive1, MotorType.kBrushless);
      rightDrive1.setInverted(false);
    rightDrive2 = new CANSparkMax (Constants.krightDrive2, MotorType.kBrushless);
      rightDrive2.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
