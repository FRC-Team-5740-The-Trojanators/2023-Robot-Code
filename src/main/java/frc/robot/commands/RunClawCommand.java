// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.VisionTargeting;

public class RunClawCommand extends CommandBase 
{ 
  private final Claw m_claw; 
  private boolean m_isFinished;
  private String m_function;
  private String m_objectType;
  private VisionTargeting m_VisionTargeting = new VisionTargeting();

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
    m_isFinished = m_claw.getTemperatureError();
    m_objectType = m_VisionTargeting.getObjectType();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    m_objectType = m_VisionTargeting.getObjectType();
    //SmartDashboard.putBoolean("Object Type cone", m_objectType.contentEquals("cone"));
    //SmartDashboard.putBoolean("Object Type cube", m_objectType.contentEquals("cube"));
    //SmartDashboard.putBoolean("function", m_function == "FORWARD");
    //SmartDashboard.putBoolean("cone threshold", (m_claw.getRange() < Constants.ClawSubsystemConstants.k_coneThreshold));
    //SmartDashboard.putBoolean("cube threshold", (m_claw.getRange() < Constants.ClawSubsystemConstants.k_cubeThreshold));

    if (m_function == "FORWARD")
    {
      if ((m_objectType.contentEquals("cone") && (m_claw.getRange() < Constants.ClawSubsystemConstants.k_coneThreshold)) || 
          (m_objectType.contentEquals("cube") && (m_claw.getRange() < Constants.ClawSubsystemConstants.k_cubeThreshold)))
      {    
        m_claw.holdClawMotor();
      }
      else
      {
        m_claw.forwardClawMotor();
      }
    }

    if (m_function == "BACKWARD") 
    {
      m_claw.reverseClawMotor();
    }

    if (m_function == "STOP") 
    {
      m_claw.stopClawMotor();
    }

    m_isFinished = m_claw.getTemperatureError();
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
