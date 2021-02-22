/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Elevator extends SubsystemBase {
  private final VictorSPX  m_elevator1;
  private final CANSparkMax m_elevator2;
  public Elevator() {
    m_elevator1 = new VictorSPX(Constants.ELEVATOR1);
    m_elevator2 = new CANSparkMax(Constants.ELEVATOR2, MotorType.kBrushless);

    m_elevator1.configFactoryDefault();
    m_elevator2.restoreFactoryDefaults();

    m_elevator1.setInverted(InvertType.None);
    m_elevator2.setInverted(false);

    m_elevator1.setNeutralMode(NeutralMode.Brake);
    m_elevator2.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public synchronized void extendLift(){
    m_elevator1.set(ControlMode.PercentOutput, .7);
  }

  public synchronized void wingardiumLeviosa(){
    m_elevator1.set(ControlMode.PercentOutput, -.7);
    m_elevator2.set(1);
  }
}
