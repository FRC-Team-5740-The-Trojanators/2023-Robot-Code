// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase 
{
  private CANSparkMax m_wristMotor = new CANSparkMax(Constants.CANBusIDs.k_wristMotorID, MotorType.kBrushless);
  private PIDController m_wristMotorPID = new PIDController(Constants.MotorPIDValues.k_wristMotorP, Constants.MotorPIDValues.k_wristMotorI, Constants.MotorPIDValues.k_wristMotorD);
  private DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_wristEncoderPort);
  
  /** Creates a new Wrist. */
  public Wrist()
  {
    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.enableVoltageCompensation(10);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotorPID.setTolerance(Constants.MotorPIDValues.k_wristTolerance);
    m_wristMotorPID.disableContinuousInput();
    m_wristMotor.setInverted(false);
    m_wristMotor.setOpenLoopRampRate(1);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Abs Encoder", getAbsEncoder());
    SmartDashboard.putData(m_wristMotorPID);
  }

  public double getAbsEncoder()
  {
      return m_wristEncoder.get();
  }

  public double setSetpoint(double posSetPoint)
  {
    //return m_shoulderMotorPID.calculate(getAbsEncoder(), new TrapezoidProfile.State(posSetPoint, 0), new TrapezoidProfile.Constraints(0.01, .1));
    return m_wristMotorPID.calculate(getAbsEncoder(), posSetPoint);
  }

  public boolean moveEnd()
  {
    return m_wristMotorPID.atSetpoint();
  }

  public void setMotor(double demand)
  {       
    m_wristMotor.set(demand);    
  }

  public void forceMotorExtend()
  {
      m_wristMotor.set(.1);
  }

  public void forceMotorRetract()
  {
    m_wristMotor.set(-0.1);
  }

  public void forceMotorStop()
  {
    m_wristMotor.set(0);
  }
}
