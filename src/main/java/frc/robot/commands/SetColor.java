// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetColor extends CommandBase 
{
  LEDColor m_red; 
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  LEDColor color;

  public class LEDColor 
  {
    private int r;
    private int g;
    private int b;

    public LEDColor(int r, int g, int b)
    {
        this.r = r;
        this.g = g;
        this.b = b;
        m_red = new LEDColor(255, 0, 0);
    }

    public int getR()
    {
        return r;
    }
    public int getG()
    {
        return g;
    }
    public int getB()
    {
        return b;
    }
  }
  /** Creates a new LEDs. */
  public SetColor() 
  {
    this.color = color;
    addRequirements(RobotContainer.m_leds);
    
    //public static final LEDColor m_red = new LEDColor(255, 0, 0);
    /*public static final LEDColor kGreen = new LEDColor(0, 255, 0);
    public static final LEDColor kBlue = new LEDColor(0, 0, 255);
    public static final LEDColor kYellow = new LEDColor(255, 255, 0);
    public static final LEDColor kPurple = new LEDColor(127, 0, 255);
    public static final LEDColor kOrange = new LEDColor(255, 95, 31);*/

    //public static final LEDColor kOff = new LEDColor(0, 0, 0);
    }

    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    public static SetColor(LEDColor color)
    {
      for(int i = 0; i < m_ledBuffer.getLength(); i++){
        m_ledBuffer.setRGB(i, color.getR(), color.getG(), color.getB()); 
      }

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    RobotContainer.m_leds.setColor(color);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
  
}
