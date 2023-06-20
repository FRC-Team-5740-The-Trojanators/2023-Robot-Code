// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SetColorValues;
import frc.robot.subsystems.LEDs;
import lib.LEDColor;

public class SetColor extends CommandBase 
{
  private String m_color;
  private LEDs m_leds;
  private Timer m_timer = new Timer();
  /* private SetColorValues kPurple; 
  private SetColorValues kYellow; */
  /** Creates a new LEDs. */
  public SetColor(LEDs led, String color) 
  {
    m_leds = led;
    m_color = color;
    addRequirements(m_leds);
    }

    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_timer.reset();
    m_timer.start();

    if(m_color.contentEquals("purple"))
    {
      //SmartDashboard.putString("PURPLE", "ON");
      //m_leds.setRGBColor(new LEDColor(255,  0, 0));
      m_leds.setRGBDualColor(new LEDColor(255,  0, 0), new LEDColor(0,  255, 0));
    } 
    else
    { //SmartDashboard.putString("PURPLE", "Off");
  }
    if(m_color.contentEquals("yellow"))
    {
      //SmartDashboard.putString("YELLOW", "ON");
      //for(int i = 0; i < Constants.LEDsSubsystemConstants.k_numLeds; i++)
       //{
        m_leds.setRGBColor(new LEDColor(255, 255, 0));
        //}
    } 
    else
    {
      //SmartDashboard.putString("YELLOW", "OFF");
    }

    if(m_color.contentEquals("off"))
    {
      //SmartDashboard.putString("OFF", "ON");
      for(int i = 0; i < Constants.LEDsSubsystemConstants.k_numLeds; i++)
        {
        m_leds.setRGBColor(new LEDColor(0, 0, 0));
        }
    } 
    else
    {
      //SmartDashboard.putString("OFF", "OFF");
    }
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    for(int i = 0; i < Constants.LEDsSubsystemConstants.k_numLeds; i++)
    {
    m_leds.setRGBColor(new LEDColor(0, 0, 0));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return m_timer.get() > 10;
  }
}

