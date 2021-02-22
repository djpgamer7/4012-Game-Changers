/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * @author Daniel Pearson
 * @version 1/9/2020
 */
public class TankDrive extends CommandBase {
  private static DriveTrain drive;
  private static DoubleSupplier leftSpeed;
  private static DoubleSupplier rightSpeed;

  public TankDrive(DriveTrain subsystem, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    drive = subsystem;
    leftSpeed = leftSupplier;
    rightSpeed = rightSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    drive.manualDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    drive.manualDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
