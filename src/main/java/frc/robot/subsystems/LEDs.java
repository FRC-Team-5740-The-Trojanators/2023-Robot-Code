


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetColor;
import lib.LEDColor;


public class LEDs extends SubsystemBase 
{
  //Creates a new LEDs. 
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private boolean isOn;
  private double lastChange;
  private int m_port;
  private int m_numLeds;

  public LEDs(int port, int numLeds)
  {    
    m_port = port;
    m_numLeds = numLeds;
    m_led = new AddressableLED(m_port);
   
    m_ledBuffer = new AddressableLEDBuffer(m_numLeds);
    
    m_led.setLength(Constants.LEDsSubsystemConstants.k_numLeds);
    //m_led.setBitTiming(0, 1, 0, 1);

    for(int i = 0; i< m_ledBuffer.getLength() ; i++)
    {
    m_ledBuffer.setRGB(i, 0, 0, 0);
    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

   public void setRGBColor(LEDColor color)
  {
    for(int i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, color.getR(), color.getG(), color.getB()); 
    }
    m_led.setData(m_ledBuffer);
  }

  public void setHSVColor(int hue, int saturation, int value)
  {
    for(int i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setHSV(i, hue, saturation, value); 
    }
    m_led.setData(m_ledBuffer);
  }

  public void setRGBDualColor(LEDColor color1, LEDColor color2)
  {
    for(int i = 0; i < m_ledBuffer.getLength(); i++)
    {
      if( i % 2 == 0)
      {
        m_ledBuffer.setRGB(i, color1.getR(), color1.getG(), color1.getB()); 
      }
      else
      {
        m_ledBuffer.setRGB(i, color2.getR(), color2.getG(), color2.getB()); 
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void pulse(LEDColor color, double interval)
  {
    double timestamp = Timer.getFPGATimestamp();
    if(timestamp -lastChange > interval)
    {
      lastChange = timestamp;
      isOn = !isOn;
    }
    if(isOn)
    {
      stop();
    }
    else
    {
      setRGBColor(color);
    }
  }

  public void stop()
  {
    setRGBColor(Constants.SetColorValues.kOff);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run\
  }
}




