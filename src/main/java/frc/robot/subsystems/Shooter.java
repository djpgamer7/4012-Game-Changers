/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * @author Daniel Pearson 
 * @version 3/2/2020
 */
public class Shooter extends SubsystemBase {
  private final TalonFX m_shooter;

  private double targetVelocityPer100ms = Constants.shooterSetPoint * 500.0 * 4096 / 600;

  public Shooter() {

    m_shooter = new TalonFX(Constants.SHOOTER_MOTOR);

    m_shooter.configFactoryDefault();

    m_shooter.setInverted(InvertType.None);
    m_shooter.setNeutralMode(NeutralMode.Coast);

    m_shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.shooterTimeout);

    m_shooter.setSensorPhase(false);

    m_shooter.config_kP(0, Constants.shooterkP, Constants.shooterTimeout);
    m_shooter.config_kI(0, Constants.shooterkI, Constants.shooterTimeout);
    m_shooter.config_kD(0, Constants.shooterkD, Constants.shooterTimeout);

    

    m_shooter.configNominalOutputForward(0, Constants.shooterTimeout);
    m_shooter.configNominalOutputReverse(0, Constants.shooterTimeout);
    m_shooter.configPeakOutputForward(0.8, Constants.shooterTimeout);
    m_shooter.configPeakOutputReverse(-0.8, Constants.shooterTimeout);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getRpm(){
    return m_shooter.getSensorCollection().getIntegratedSensorVelocity();
  }

  public synchronized void activateClosedLoopControl(){
    m_shooter.set(ControlMode.Velocity, targetVelocityPer100ms);
  }

  public synchronized void setVelocity(double velocity){
    double setVelocity = velocity * 500 * 4096 / 600;
    m_shooter.set(ControlMode.Velocity, setVelocity);
  }

  public synchronized void setPercent(double percent){
    m_shooter.set(ControlMode.PercentOutput, percent);
  }
  
  public boolean atSetpoint(){
    if(this.getRpm() == Constants.shooterSetPoint){
      return true;
    }
    return false;
  }
}
