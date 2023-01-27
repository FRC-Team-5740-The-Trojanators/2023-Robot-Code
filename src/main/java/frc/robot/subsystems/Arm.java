// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase 
{
  /** Creates a new Arm. */

  CANSparkMax m_masterShoulderMotor = new CANSparkMax(Constants.CANBusIDs.k_masterShoulderMotorID, MotorType.kBrushless);
  CANSparkMax m_followerShoulderMotor = new CANSparkMax(Constants.CANBusIDs.k_followerShoulderMotorID, MotorType.kBrushless);
  CANSparkMax m_wristMotor = new CANSparkMax(Constants.CANBusIDs.k_wristMotorID, MotorType.kBrushless);

  DutyCycleEncoder m_shoulderEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_shoulderEncoderPort);
  DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_wristEncoderPort);

  public Arm()
  {
    m_masterShoulderMotor.restoreFactoryDefaults();
    m_followerShoulderMotor.restoreFactoryDefaults();
    m_wristMotor.restoreFactoryDefaults();

    m_followerShoulderMotor.follow(m_masterShoulderMotor);
    m_masterShoulderMotor.enableVoltageCompensation(10);
  }

//TODO: Create configure SparkMax method and call it 3 times

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
