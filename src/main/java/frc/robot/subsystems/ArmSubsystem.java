// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX armMotor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor = new TalonFX(0); // Check ID //
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armMotorUp(XboxController xboxcontroller) {
    armMotor.set(ControlMode.PercentOutput, xboxcontroller.getPOV(90));
  }

  public void armMotorUDown(XboxController xboxcontroller) {
    armMotor.set(ControlMode.PercentOutput, xboxcontroller.getPOV(270));
  }
}
