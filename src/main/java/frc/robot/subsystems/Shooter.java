/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static CANSparkMax shooterMotor;
  public static CANEncoder shooterEnc;
  public static SmartDashboard dashboard;
  private static boolean atSetpoint;


  private double targetVelocityPer100ms = Constants.shooterSetPoint * 500.0 * 4096 / 600;

  public Shooter() {
    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    shooterMotor.clearFaults();
    shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterEnc.setInverted(false);

    shooterEnc = shooterMotor.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getRpm());
    SmartDashboard.putNumber("Shooter Pos", getPosition());
  }

  public double getRpm() {
    return shooterEnc.getCountsPerRevolution();
  }

  public double getPosition() {
    return shooterEnc.getPosition();
  }

  public synchronized void setVelocity(double velocity) {
    double shooterVal = 0;
    if(shooterEnc.getVelocity() < velocity) {
      double diff = shooterEnc.getVelocity() - velocity;

      shooterVal = diff * Constants.shooterkP;
      if(shooterVal > .7) {
        shooterVal = .7;
      }
    } else if(shooterEnc.getVelocity() > velocity) {
      double diff = velocity - shooterEnc.getVelocity();

      shooterVal = diff * Constants.shooterkP;

      if(shooterVal < -.7) {
        shooterVal = -.7;
      }
    }

    SmartDashboard.putNumber("Shooter Val", shooterVal);

    shooterMotor.set(shooterVal);
  }

  public synchronized void setSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public boolean atSetpoint() {
    if(this.getRpm() == Constants.shooterSetPoint) {
      atSetpoint = true;
      return true;
    }
    atSetpoint = false;
    return false;
  }
  /*

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
<<<<<<< HEAD
  }*/
}

