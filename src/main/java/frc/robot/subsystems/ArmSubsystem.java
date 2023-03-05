// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceMode;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax armMotor;
  private Encoder armAbsoluteEncoder;
  // private double armEncoderDistance;
  private SparkMaxPIDController armPIDController;
  // private double m_setpoint = 0;
  // private double pidReference = 0;
  private double armPValue = 0;
  private double armDValue = 0;

  private SparkMaxLimitSwitch forwardLimitSwitch;
  private SparkMaxLimitSwitch backwardLimitSwitch;

  private boolean isManualEnabled = false;

  /** Creates a new ArmSystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.Arm.karmMotor, MotorType.kBrushless);
    armAbsoluteEncoder = new Encoder(2, 1, true, CounterBase.EncodingType.k4X);
    forwardLimitSwitch = armMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    backwardLimitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    armPIDController = armMotor.getPIDController();

    armPIDController.setP(0.1);
    // armPIDController.setI(kI);
    armPIDController.setD(10);
    // armPIDController.setIZone(kIz);
    // armPIDController.setFF(kFF);
    armPIDController.setOutputRange(-1, 1);

    armMotor.setIdleMode(IdleMode.kBrake);

    armMotor.setOpenLoopRampRate(0.35);
    armMotor.setClosedLoopRampRate(0.35);

    armMotor.setSmartCurrentLimit(40);

    armMotor.getEncoder().setPositionConversionFactor(1);

    // SmartDashboard.putNumber("Arm P Value", 0.1);
    // SmartDashboard.putNumber("Arm D Value", 0.01);

    // SmartDashboard.putNumber("PID Set Reference", pidReference);

  }

  @Override
  public void periodic() {

    // always display current position of arm
    SmartDashboard.putNumber("Arm Height", armMotor.getEncoder().getPosition());
    

    // armPValue = SmartDashboard.getNumber("Arm P Value", 0);
    // armDValue = SmartDashboard.getNumber("Arm D Value", 0);

    // armPIDController.setP(armPValue);
    // armPIDController.setD(armDValue);

    // reset all encoders and PID position when lower limit switch is pressed
    if (backwardLimitSwitch.isPressed()) {
      armAbsoluteEncoder.reset();
      armMotor.getEncoder().setPosition(0);
      armPIDController.setReference(0, ControlType.kPosition);
    }

    // manual movement of arm using arrow keys on button box
    if (RobotContainer.m_buttonBox.getPOV() == 0) {
      isManualEnabled = true;
      armMotor.set(0.6);
    } else if (RobotContainer.m_buttonBox.getPOV() == 180) {
      isManualEnabled = true;
      armMotor.set(-0.4);
    } else if (isManualEnabled) {
      isManualEnabled = false;
      armMotor.set(0);
    }

    // reset height
    if (RobotContainer.m_buttonBox.getXButton()) {
      goToHomePosition();
    }

    // carry height
    if (RobotContainer.m_buttonBox.getYButton()) {
      goToCarryPosition();
    }

    // mid height for cone or cube based on mode
    if (RobotContainer.m_buttonBox.getRightBumper()) {
      goToMidPosition();
    }

    // high height for cone or cube based on mode
    if (RobotContainer.m_buttonBox.getLeftBumper()) {
      goToHighPosition();
    }

    // if (RobotContainer.m_buttonBox.getAButton()) {
    // pidReference = SmartDashboard.getNumber("PID Set Reference", 0);
    // armPIDController.setReference(pidReference,
    // CANSparkMax.ControlType.kPosition);
    // }

    // This method will be called once per scheduler run

  }

  public void armMotorSet(double speed) {
    armMotor.set(speed);
  }

  public void resetArmEncoders() {
    armAbsoluteEncoder.reset();
  }

  // public void armUpAuto(double speed, double setpoint) {
  // if (armAbsoluteEncoder.getDistance() > setpoint + 30) { // go up
  // armMotorSet(speed);
  // } else if (armAbsoluteEncoder.getDistance() < setpoint - 30) { // go down
  // armMotorSet(-speed);
  // } else {
  // armMotorSet(0);
  // }
  // }

  public void goToHomePosition() {
    armPIDController.setReference(Constants.Setpoints.kresetSetpoint, ControlType.kPosition);
  }

  public void goToCarryPosition() {
    armPIDController.setReference(Constants.Setpoints.kcarrySetpoint, ControlType.kPosition);
  }

  public void goToMidPosition() {
    armPIDController.setReference(
        RobotContainer.gamePieceMode == GamePieceMode.CONE ? Constants.Setpoints.kconeMidSetpoint
            : Constants.Setpoints.kcubeMidSetpoint,
        ControlType.kPosition);
  }

  public void goToHighPosition() {
    armPIDController.setReference(
        RobotContainer.gamePieceMode == GamePieceMode.CONE ? Constants.Setpoints.kconeHighSetpoint
            : Constants.Setpoints.kcubeHighSetpoint,
        ControlType.kPosition);
  }

  public void goToPosition(double position) {
    armPIDController.setReference(position, ControlType.kPosition);
  }

  public double getArmEncoderDistance() {
    return armMotor.getEncoder().getPosition();
  }

}
