// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrvConst;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  private final SwerveModule frontLeft = new SwerveModule(DrvConst.frontLeft);
  private final SwerveModule frontRight = new SwerveModule(DrvConst.frontRight);
  private final SwerveModule backLeft = new SwerveModule(DrvConst.backLeft);
  private final SwerveModule backRight = new SwerveModule(DrvConst.backRight);

  private final  AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          DrvConst.frontLeftLocation, DrvConst.frontRightLocation, DrvConst.backLeftLocation, DrvConst.backRightLocation);

  private final SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(
          kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          },
          Pose2d.kZero);
  public Drivetrain() {
    gyro.reset();//TODO: Is this line needed?
    while(gyro.isCalibrating());
    zeroYaw();
  }

  void zeroYaw() {
    gyro.zeroYaw();
  }

  void setRelOffset() {
    frontLeft.setRelOffset();
    frontRight.setRelOffset();
    backLeft.setRelOffset();
    backRight.setRelOffset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
  Translation2d FLVel, BLVel, FRVel, BRVel,  
  BaseVel = new Translation2d(xSpeed, ySpeed);
  double maxSpeed = BaseVel.getNorm() + DrvConst.driveRadius * Math.abs(rot);
  if (maxSpeed > DrvConst.overloadSpeed) {
    BaseVel = BaseVel.times(DrvConst.overloadSpeed / maxSpeed);
    rot *= (DrvConst.overloadSpeed / maxSpeed);
  }

  if(fieldRelative) {
    BaseVel = BaseVel.rotateBy(gyro.getRotation2d().unaryMinus());
  }
  FLVel = BaseVel.plus(DrvConst.frontLeftClW.times(rot));
  BLVel = BaseVel.plus(DrvConst.backLeftClW.times(rot));
  FRVel = BaseVel.plus(DrvConst.frontRightClW.times(rot));
  BRVel = BaseVel.plus(DrvConst.backRightClW.times(rot));
  
  frontLeft.setVel(FLVel);
  backLeft.setVel(BLVel);
  frontRight.setVel(FRVel);
  backRight.setVel(BRVel);
}

void setXPosture() {
  Translation2d FLVel = DrvConst.frontLeftLocation.div(128),
                BLVel = DrvConst.backLeftLocation.div(128), 
                FRVel = DrvConst.frontRightLocation.div(128), 
                BRVel = DrvConst.backRightLocation.div(128);
  frontLeft.setVel(FLVel);
  backLeft.setVel(BLVel);
  frontRight.setVel(FRVel);
  backRight.setVel(BRVel);
}

void fullSpeed() {
  backLeft.fullSpeed();
  backRight.fullSpeed();
  frontLeft.fullSpeed();
  frontRight.fullSpeed();
}
  /** Display debugging info */
   void report() {
    frontLeft.report();
    frontRight.report();
    backLeft.report();
    backRight.report();
    var pose = odometry.getEstimatedPosition();
    SmartDashboard.putNumber("estd. X", pose.getX());
    SmartDashboard.putNumber("estd. Y", pose.getY());
    SmartDashboard.putNumber("estd. Angle", pose.getRotation().getRotations());
  }
  /** base future odometry at {@code currentPose}
   * @param currentPose the "known" current Pose2d
   */
  void resetOdometry(Pose2d currentPose) {
    odometry.resetTranslation(currentPose.getTranslation());
  }

  /** @return where it thinks we are */
  Pose2d reportOdometry() {
    return odometry.getEstimatedPosition();
  }
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }
}
