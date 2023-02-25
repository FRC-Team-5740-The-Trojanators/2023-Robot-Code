// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class RunClawCommand extends CommandBase 
{ 
  private final Claw m_claw; 
  private boolean m_isFinished;
  private String m_function;

  /** Creates a new RunClawCommand. */
  public RunClawCommand(Claw claw, String function) 
  {
    m_claw = claw;
    m_function = function;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_function == "FORWARD")
    {
      m_claw.forwardClawMotor();
    }
    if (m_function == "BACKWARD")
    {
      m_claw.reverseClawMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_isFinished = true;
    m_claw.stopClawMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return m_isFinished;
  }
}
