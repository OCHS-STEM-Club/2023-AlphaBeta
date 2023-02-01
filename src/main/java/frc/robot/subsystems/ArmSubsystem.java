// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

  private TalonSRX armMotor;
  /** Creates a new ArmSystem. */
  public ArmSubsystem() {
    armMotor = new TalonSRX(Constants.Arm.karmMotor);
  }

  @Override
  public void periodic() {
    double outputSpeed = 0;
    if (RobotContainer.m_operatorController.getLeftBumper()) {
      outputSpeed += 0.25;
    }

    if (RobotContainer.m_operatorController.getRightBumper()) {
      outputSpeed -= 0.25;
    }

    armMotorSet(outputSpeed);

    
    // This method will be called once per scheduler run
  }

  public void armMotorSet(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

}
