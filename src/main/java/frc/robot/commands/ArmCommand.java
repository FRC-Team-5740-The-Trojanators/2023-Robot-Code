// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private Shoulder m_shoulder;
  private Wrist m_wrist;
  private String m_position;
  
  private boolean m_isFinished;
  private double m_shoulderSetPoint;
  private double m_wristSetPoint;

  public ArmCommand(Shoulder shoulder, Wrist wrist, String Position)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = shoulder;
    m_wrist = wrist;
    m_position = Position;

    addRequirements(m_shoulder, m_wrist);
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
    if (!m_shoulder.moveEnd())
    {
      if (m_position == "STOWED")
      {
        m_shoulderSetPoint = ArmPositionConstants.shoulderStowed;
        m_wristSetPoint = ArmPositionConstants.wristStowed;
      }
      else if (m_position == "TOPGRID")
      {
        m_shoulderSetPoint = ArmPositionConstants.shoulderTopGrid;
        m_wristSetPoint = ArmPositionConstants.wristTopGrid;
      }
      else if (m_position == "MIDGRID")
      {
        m_shoulderSetPoint = ArmPositionConstants.shoulderMidGrid;
        m_wristSetPoint = ArmPositionConstants.wristMidGrid;
      }
      else if (m_position == "FLOOR")
      {
        m_shoulderSetPoint = ArmPositionConstants.shoulderFloor;
        m_wristSetPoint = ArmPositionConstants.wristFloor;
      }
    }

    m_shoulder.setMotor(m_shoulder.setSetpoint(m_shoulderSetPoint));
    m_wrist.setMotor(m_wrist.setSetpoint(m_wristSetPoint));

    m_isFinished = (m_shoulder.moveEnd() && m_wrist.moveEnd());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_isFinished = true;
    m_shoulder.forceMotorStop();
    m_wrist.forceMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_isFinished;
  }
}
