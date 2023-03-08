// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ExtendClawCommand extends CommandBase 
{
  /** Creates a new ExtendClawCommand. */
  Claw m_claw; 
  private boolean m_isFinished = false;
  public ExtendClawCommand(Claw claw) 
  {
    m_claw = claw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

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
    m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return m_isFinished;
  }
}
