// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPositionConstants;

public class Shoulder extends SubsystemBase 
{
  /** Creates a new Shoulder. */
  private double m_lastSpeed = 0;
  private double m_lastTime = Timer.getFPGATimestamp();
  private double m_setpoint = ArmPositionConstants.shoulderStowed;

  private MedianFilter m_filter = new MedianFilter(16);
  private double m_filteredAngle;
  private boolean m_shoulderEncoderFlag = false;

  private final TrapezoidProfile.Constraints m_trapConstraints = new TrapezoidProfile.Constraints(2, 5);
  private CANSparkMax m_shoulderMotor = new CANSparkMax(Constants.CANBusIDs.k_shoulderMotorID, MotorType.kBrushless);
  private SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(Constants.MotorPIDValues.k_shoulderMotorFF_Ks,Constants.MotorPIDValues.k_shoulderMotorFF_Kv , Constants.MotorPIDValues.k_shoulderMotorFF_Ka);
  private ProfiledPIDController m_shoulderMotorPID = new ProfiledPIDController(Constants.MotorPIDValues.k_shoulderMotorP, Constants.MotorPIDValues.k_shoulderMotorI, Constants.MotorPIDValues.k_shoulderMotorD, m_trapConstraints);
  private DutyCycleEncoder m_shoulderEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_shoulderEncoderPort);

  public Shoulder()
  {
    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderMotor.enableVoltageCompensation(10);
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderMotor.setInverted(false);
    m_shoulderMotor.setSmartCurrentLimit(40);
    m_shoulderMotorPID.setTolerance(Constants.MotorPIDValues.k_shoulderTolerance);
    m_shoulderMotorPID.disableContinuousInput();
    m_shoulderMotorPID.setTolerance(.1, 1);
    setSetpoint(ArmPositionConstants.shoulderStowed);
  }

  @Override
  public void periodic() 
  {
    if(m_shoulderEncoder.isConnected()) m_shoulderEncoderFlag = true;

    if((getAngleRadians() < -.25) || (getAngleRadians() > 1.55))
    {
      m_shoulderEncoderFlag = false;
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Abs Encoder", getAbsEncoder());
    SmartDashboard.putNumber("Shoulder Angle Radians", getAngleRadians());
    //SmartDashboard.putNumber("error", m_shoulderMotorPID.getPositionError());
    SmartDashboard.putNumber("Setpoint Pos", m_shoulderMotorPID.getSetpoint().position);
    //SmartDashboard.putNumber("Setpoint Vel", m_shoulderMotorPID.getSetpoint().velocity);

    for(int i = 0; i < 16; i++)
    {
      m_filteredAngle = m_filter.calculate(getAngleRadians());
    }

    goToPosition(m_setpoint);
  }

// Controls a simple motor's position using a SimpleMotorFeedforward
// and a ProfiledPIDm_shoulderMotorPID
  public void goToPosition(double goalPosition) 
  {
    //System.out.println("go to position, yes");
    double pidVal = m_shoulderMotorPID.calculate(m_filteredAngle, goalPosition);
    double acceleration = (m_shoulderMotorPID.getSetpoint().velocity - m_lastSpeed) / (Timer.getFPGATimestamp() - m_lastTime);

    if(m_shoulderEncoderFlag)
    {
      m_shoulderMotor.setVoltage(pidVal + m_armFeedforward.calculate(m_shoulderMotorPID.getSetpoint().velocity, acceleration));
      m_lastSpeed = m_shoulderMotorPID.getSetpoint().velocity;
      m_lastTime = Timer.getFPGATimestamp();
    }
    else
    {
      //System.out.println("shoulder motor stop");
      forceMotorStop();
    }
  }

  public void setSetpoint(double setpoint)
  {
    m_setpoint = setpoint;
  }

  public void setInitialSetPoint()
  {
    m_shoulderMotorPID.reset(getAngleRadians());
  }

  public boolean moveEnd()
  {
    return m_shoulderMotorPID.atSetpoint();
  }

  public void forceMotorExtend()
  {
      m_shoulderMotor.set(.1);
  }

  public void forceMotorRetract()
  {
    m_shoulderMotor.set(-0.1);
  }

  public void forceMotorStop()
  {
    m_shoulderMotor.set(0);
  }

  public double getAngleRadians()
  {
      return ((m_shoulderEncoder.get() - Constants.ArmPositionConstants.shoulderOffset) * Math.PI * 2);
  }

  public double getAbsEncoder()
  {
      return m_shoulderEncoder.get();
  }

  public double getFilteredAngle()
  {
      return m_filteredAngle;
  }
}
