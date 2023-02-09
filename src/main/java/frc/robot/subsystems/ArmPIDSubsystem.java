// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.CounterBase;

public class ArmPIDSubsystem extends PIDSubsystem {
  private TalonSRX armMotor = new TalonSRX(Constants.Arm.karmMotor);
  private Encoder armEncoder = new Encoder(2, 1, true, CounterBase.EncodingType.k4X);

  /** Creates a new ArmPIDSubsystem. */
  public ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1.6878, 0, 0));
        getController().setTolerance(30);
        //armEncoder.setDistancePerPulse(); // TODO: Check if needed//
        setSetpoint(-300);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    armMotor.set(ControlMode.Position, output + m_controller.calculate(setpoint)); //TODO: Check if right //
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return armEncoder.getDistance();
    
  }

  public boolean atSetPoint() {
    return m_controller.atSetpoint();
  }

  public void encoderValue() {
    System.out.println(armEncoder.getDistance());
    
  }

}
