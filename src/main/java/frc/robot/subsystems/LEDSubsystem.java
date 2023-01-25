// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.security.auth.x500.X500Principal;

import com.revrobotics.ColorSensorV3.LEDCurrent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_led = new AddressableLED(Constants.LED.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LED.BUFFERSIZE);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setFrontAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  public void setBallLEDs(XboxController xboxController) {
    if (xboxController.getBButtonPressed() == true) {
      System.out.print("B Button Pressed");
      setFrontAll(Color.kRed);
    } else if (xboxController.getAButtonPressed() == true) {
      System.out.print("A Button Pressed");
      setFrontAll(Color.kGreen);
    } else {
      setFrontAll(Color.kBlue); 
    }
  }

  public void setBall(Color color) {
    // m_ledBuffer.setFrontAll(Color.kBlue); 
  }
}

