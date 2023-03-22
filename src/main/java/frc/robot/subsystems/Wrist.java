// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorPIDValues;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants;

public class Wrist extends SubsystemBase 
{
  private double m_setpoint = ArmPositionConstants.wristStowed;
  
  private MedianFilter m_filter = new MedianFilter(16);
  private double m_filteredAngle;
  private boolean m_wristEncoderFlag = false;

  //private final TrapezoidProfile.Constraints m_trapConstraints = new TrapezoidProfile.Constraints(.75, .75);
  //private ArmFeedforward m_armFeedforward = new ArmFeedforward(MotorPIDValues.k_wristMotorFF_Ks, MotorPIDValues.k_wristMotorFF_Kg , MotorPIDValues.k_wristMotorFF_Kv);
  //private SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(MotorPIDValues.k_wristMotorFF_Ks, MotorPIDValues.k_wristMotorFF_Kv , MotorPIDValues.k_wristMotorFF_Ka);
  //private ProfiledPIDController m_wristMotorPID = new ProfiledPIDController(MotorPIDValues.k_wristMotorP, MotorPIDValues.k_wristMotorI, MotorPIDValues.k_wristMotorD, m_trapConstraints);
  private PIDController m_wristMotorPID = new PIDController(MotorPIDValues.k_wristMotorP, MotorPIDValues.k_wristMotorI, MotorPIDValues.k_wristMotorD);
  private CANSparkMax m_wristMotor = new CANSparkMax(CANBusIDs.k_wristMotorID, MotorType.kBrushless);
  private DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_wristEncoderPort);
  
  /** Creates a new Wrist. */
  public Wrist()
  {
    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.enableVoltageCompensation(10);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotorPID.setTolerance(MotorPIDValues.k_wristTolerance);
    m_wristMotorPID.disableContinuousInput();
    m_wristMotor.setInverted(false);
    m_wristMotor.setOpenLoopRampRate(2);
    m_wristMotor.setSmartCurrentLimit(80);
    setSetpoint(ArmPositionConstants.wristStowed);
  }

  @Override
  public void periodic() 
  {
    if(m_wristEncoder.isConnected()) m_wristEncoderFlag = true;

    if((getAngleRadians() < -1.59) || (getAngleRadians() > 2.75))
    {
      m_wristEncoderFlag = false;
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Abs Encoder", getAbsEncoder());
    SmartDashboard.putNumber("Wrist Angle Radians", getAngleRadians());
    //SmartDashboard.putData(m_wristMotorPID);
    //SmartDashboard.putNumber("Wrist error", m_wristMotorPID.getPositionError());
    SmartDashboard.putNumber("Wrist Setpoint Pos", m_wristMotorPID.getSetpoint());
    //SmartDashboard.putNumber("Wrist Setpoint Vel", m_wristMotorPID.getSetpoint().velocity);
      
    for(int i = 0; i < 16; i++)
    {
      m_filteredAngle = m_filter.calculate(getAngleRadians());
    }

    goToPosition(m_setpoint);
  }

  public void goToPosition(double goalPosition)
  {
    double pidVal = m_wristMotorPID.calculate(m_filteredAngle, goalPosition);
    
    if(m_wristEncoderFlag)
    {
       m_wristMotor.setVoltage(pidVal);
    }
    else
    {
      forceMotorStop();
    }
  }

  public void setSetpoint(double setpoint)
  {
    m_setpoint = setpoint;
  }

  public void setInitialSetPoint()
  {
    //m_wristMotorPID.reset(m_filteredAngle);
  }

  public double getAngleRadians()
  {
    double angle = ((m_wristEncoder.get() - Constants.ArmPositionConstants.wristOffset) * Math.PI * 2);
    if ((angle < -Math.PI) && (angle > (-Math.PI * 3)))
    {
      return (angle + (Math.PI * 2));
    }
    else
    {
      return angle;
    }
  }

  public double getAbsEncoder()
  {
      return m_wristEncoder.get();
  }

  public boolean moveEnd()
  {
    return m_wristMotorPID.atSetpoint();
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
