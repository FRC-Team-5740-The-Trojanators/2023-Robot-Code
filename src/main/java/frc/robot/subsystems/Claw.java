// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawSubsystemConstants;

public class Claw extends SubsystemBase 
{
  private CANSparkMax m_clawMotor;

  /** Creates a new Claw. */
  public Claw() 
  {
    m_clawMotor = new CANSparkMax(Constants.CANBusIDs.k_clawMotorID, MotorType.kBrushless);
    m_clawMotor.restoreFactoryDefaults();
    m_clawMotor.setIdleMode(IdleMode.kBrake);
  }

  // sparkmax w/ Neo550, distance sensor
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void forwardClawMotor()
  {
    m_clawMotor.set(ClawSubsystemConstants.k_clawMotorSpeed);
  }

  public void reverseClawMotor()
  {
    m_clawMotor.set(ClawSubsystemConstants.k_clawReverseMotorSpeed);
  }
  
  public void holdClawMotor()
  {
    m_clawMotor.set(ClawSubsystemConstants.k_clawHoldMotorSpeed);
  }

  public void stopClawMotor()
  {
    m_clawMotor.set(0);
  }
}
