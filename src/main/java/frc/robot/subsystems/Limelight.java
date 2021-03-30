/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  public static class LimelightConstants {
    public static final double kProt = .053;
    public static final double kPdist = .25;
    public static final double kMin = .04;
    public static final double maxSpeed = .7;
    public static final double desiredArea = 3;
  }

  public enum LedModes {
    ON, OFF, BLINK
  }

  public enum CamModes {
    VISION, DRIVER
  }

  private final NetworkTable limelightTable;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry camState;
  NetworkTableEntry ledState;

  private double xError;
  private double yError;
  private double targetArea;

  private boolean seesTarget;

  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tv = limelightTable.getEntry("tv");
    camState = limelightTable.getEntry("camMode");
    ledState = limelightTable.getEntry("ledMode");

    xError = tx.getDouble(0.0);
    yError = ty.getDouble(0.0);
    targetArea = ta.getDouble(0.0);

    seesTarget = tv.getBoolean(false);
  }

  public boolean hasTarget(){
    return seesTarget;
  }

  public double getTargetX(){
    return xError;
  }

  public double getTargetY(){
    return yError;
  }

  public double getTargetArea(){
    return targetArea;
  }

  public synchronized void setLed(LedModes mode){
    ledState.setNumber(mode.ordinal());
  }

  public synchronized void setCam(CamModes mode){
    camState.setNumber(mode.ordinal());
  }
}
