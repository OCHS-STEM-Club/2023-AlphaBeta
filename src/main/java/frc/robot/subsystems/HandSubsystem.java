// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;

public class HandSubsystem extends SubsystemBase {

  private TalonSRX handMotorLeft;
  private TalonSRX handMotorRight;
  private Ultrasonic ultrasonicSensor;
  private double ultrasonicSensorDistanceMM;
  private double ultrasonicSensorDistanceIn;
  private double time;
  public boolean gamePieceInHand;
 // public double getUltrasonicSensorDistanceIn;


  /** Creates a new HandSubsystem. */
  public HandSubsystem() {
    handMotorLeft = new TalonSRX(9); // TODO: Change ID//
    handMotorRight = new TalonSRX(10); // TODO: Change ID//
    ultrasonicSensor = new Ultrasonic(7, 8);

    

  }

  @Override
  public void periodic() {
    ultrasonicSensorDistanceIn = ultrasonicSensor.getRangeInches();

    if (ultrasonicSensorDistanceIn > 0.001 & ultrasonicSensorDistanceIn < 6) {
      gamePieceInHand = true;
   } else gamePieceInHand = false;

    // This method will be called once per scheduler run //
    // double outputSpeed = 0;
    // if (RobotContainer.m_driverController.getLeftBumper()) {
    //   outputSpeed += -0.5; // Intake //
    // } else if (ultrasonicSensorDistanceIn > 0.001 & ultrasonicSensorDistanceIn < 6) {
    //   outputSpeed += -0.5;
    //   if (RobotContainer.m_driverController.getRightBumper()) {
    //     outputSpeed += 0.5; 
    //   }
    // } else if (RobotContainer.m_driverController.getRightBumper()) {
    //   outputSpeed += 0.5; // Outake //
    // }


     //spinHandMotors(outputSpeed);

   
    //ultrasonicSensor.getRangeInches();
   
    handMotorLeft.setNeutralMode(NeutralMode.Coast);
    handMotorRight.setNeutralMode(NeutralMode.Coast);

    
    SmartDashboard.putNumber("Ultrasonic distance",  ultrasonicSensorDistanceIn);
  }

  public void spinHandMotors(double speed) {
    handMotorLeft.set(ControlMode.PercentOutput, -speed);
    handMotorRight.set(ControlMode.PercentOutput, speed);
    // System.out.println(speed);
  }

  public void setAutomaticModeUltrasonicSenor() {
    Ultrasonic.setAutomaticMode(true);
  }

  // public double getUltrasonicSensorDistanceIn(){
  //   System.out.println(ultrasonicSensor.getRangeInches());
  //   return ultrasonicSensor.getRangeInches();
  // }

  public void setAutomaticMode() {
    Ultrasonic.setAutomaticMode(true);
  }

  public void autoHandOn(double speed) {
    spinHandMotors(speed);
  }

  public boolean gamePieceInHand() {
    if (ultrasonicSensorDistanceIn < 4) {
      return true;
    } else if (ultrasonicSensorDistanceIn >= 4) {
   return false;
  } else return false;

}
}
