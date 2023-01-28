// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorPIDValues;

public class Arm extends SubsystemBase 
{
  /** Creates a new Arm. */

  CANSparkMax m_masterShoulderMotor = new CANSparkMax(Constants.CANBusIDs.k_masterShoulderMotorID, MotorType.kBrushless);
  CANSparkMax m_followerShoulderMotor = new CANSparkMax(Constants.CANBusIDs.k_followerShoulderMotorID, MotorType.kBrushless);
  CANSparkMax m_wristMotor = new CANSparkMax(Constants.CANBusIDs.k_wristMotorID, MotorType.kBrushless);
  SparkMaxPIDController m_masterShoulderMotorPID;
  SparkMaxPIDController m_followerShoulderMotorPID;
  SparkMaxPIDController m_wristMotorPID;

  DutyCycleEncoder m_shoulderEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_shoulderEncoderPort);
  DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_wristEncoderPort);

  public Arm()
  {
    m_followerShoulderMotor.follow(m_masterShoulderMotor);
    m_masterShoulderMotor.enableVoltageCompensation(10);
  
    ConfigureMotorPID();
  }

  public void ConfigureMotorPID()
  {
    m_masterShoulderMotor.restoreFactoryDefaults();
    m_followerShoulderMotor.restoreFactoryDefaults();
    m_wristMotor.restoreFactoryDefaults();

    SparkMaxPIDController m_masterShoulderMotorID = m_masterShoulderMotor.getPIDController();
    SparkMaxPIDController m_followerShoulderMotorID = m_followerShoulderMotor.getPIDController();
    SparkMaxPIDController m_wristMotorID = m_wristMotor.getPIDController();

    m_masterShoulderMotorID.setP(Constants.MotorPIDValues.k_masterShoulderMotorP);
    m_masterShoulderMotorID.setI(Constants.MotorPIDValues.k_masterShoulderMotorI);
    m_masterShoulderMotorID.setD(Constants.MotorPIDValues.k_masterShoulderMotorD);
    m_masterShoulderMotorID.setFF(Constants.MotorPIDValues.k_masterShoulderMotorFF);

    m_followerShoulderMotorID.setP(Constants.MotorPIDValues.k_followerShoulderMotorP);
    m_followerShoulderMotorID.setI(Constants.MotorPIDValues.k_followerShoulderMotorI);
    m_followerShoulderMotorID.setD(Constants.MotorPIDValues.k_followerShoulderMotorD);
    m_followerShoulderMotorID.setFF(Constants.MotorPIDValues.k_followerShoulderMotorFF);

    m_wristMotorID.setP(Constants.MotorPIDValues.k_wristMotorP);
    m_wristMotorID.setI(Constants.MotorPIDValues.k_wristMotorI);
    m_wristMotorID.setD(Constants.MotorPIDValues.k_wristMotorD);
    m_wristMotorID.setFF(Constants.MotorPIDValues.k_wristMotorFF);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
