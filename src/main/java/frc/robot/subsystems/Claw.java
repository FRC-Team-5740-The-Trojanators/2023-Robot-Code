// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase 
{
  CANSparkMax m_clawMotor = new CANSparkMax(Constants.CANBusIDs.k_clawMotorID, MotorType.kBrushless);
  SparkMaxPIDController m_clawMotorPID;
  /** Creates a new Claw. */
  public Claw() 
  {

  }

  public void ConfigureMotorPID()
  {
    m_clawMotor.restoreFactoryDefaults();

    SparkMaxPIDController m_clawMotorID = m_clawMotor.getPIDController();

    m_clawMotorID.setP(Constants.MotorPIDValues.k_clawMotorP);
    m_clawMotorID.setI(Constants.MotorPIDValues.k_clawMotorI);
    m_clawMotorID.setD(Constants.MotorPIDValues.k_clawMotorD);
    m_clawMotorID.setFF(Constants.MotorPIDValues.k_clawMotorFF);

  }
  // sparkmax w/ Neo550, distance sensor
  //Motors are all separate, will need 1 motor for this subsystem (NOT THE SAME AS ARM)
  @Override
  public void periodic() 
  { 
    
    // This method will be called once per scheduler run
  }
}
