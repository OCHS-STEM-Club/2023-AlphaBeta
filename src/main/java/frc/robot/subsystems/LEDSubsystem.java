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

  private AddressableLED m_led = new AddressableLED(Constants.LED.PWMPORT);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LED.BUFFERSIZE);
  private float hue = 0;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
   // m_led.setLength(m_ledBuffer.getLength());
    // m_led.setData(m_ledBuffer);
    // m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setFrontAll(Color color) {
    // hue = (hue + 2) % 360;
    // float period = 0.2f; // seconds
    // color = ((int)(System.currentTimeMillis() / 1000 * period) % 2 == 0) ? Color.kBlue : Color.kWhite;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
      // m_ledBuffer.setHSV(i, (int) hue, 255, 128);
    }
    m_led.setData(m_ledBuffer);
    System.out.println(m_ledBuffer.getLength());
  }

  //  public void setFrontAll(Color color) {
  //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //     m_ledBuffer.setRGB(i, 255, i, i);
  //   }
  //   m_led.setData(m_ledBuffer);
  // }

  public void setBallLEDs(XboxController xboxController) {
    System.out.print("setball intialized");
    if (xboxController.getBButton() == true) {
      System.out.print("B Button Pressed");
      setFrontAll(Color.kRed);
    } else if (xboxController.getAButton() == true) {
      System.out.print("A Button Pressed");
      setFrontAll(Color.kGreen);
    } else {
      setFrontAll(Color.kBlue); 
    }
  }

  public void initalizeLEDLength() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
}

