// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrvConst;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  // one for each wheel
  private final SwerveModule frontLeft = new SwerveModule(DrvConst.frontLeft);
  private final SwerveModule frontRight = new SwerveModule(DrvConst.frontRight);
  private final SwerveModule backLeft = new SwerveModule(DrvConst.backLeft);
  private final SwerveModule backRight = new SwerveModule(DrvConst.backRight);

  // uses a ~ gyroscope to estimate our orientation, can also measure tilt ...
  private final  AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // used by odometry, could also be used by trajectories
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          DrvConst.frontLeftLocation, DrvConst.frontRightLocation, DrvConst.backLeftLocation, DrvConst.backRightLocation);

  /** estimates location and orientation based on gyro and wheel motion */  
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

  // basic set-up, may not be needed
  public Drivetrain() {
    gyro.reset();//TODO: Is this line needed? - Phill
    //I think so because what if you have gyro on before class construction and its not hurting anything - Elliot

    // wait till gyro is ready
    while(gyro.isCalibrating());
    zeroYaw();

    // These comments are from pathplanner example vv

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    //Notes on what I did here

    //I put autobuilder.configure in the try accept for now because it said "config may not have stored value"
    //Assuming getting config works then this should totally work.
    //TODO: Do I need to put some code in periodic for this to work, or will it just work?

    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
      
    AutoBuilder.configure(
              this::reportOdometry, // Robot pose supplier
              odometry::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> drive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond,
                false // ALWAYS false for robot-relative speeds);), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              ),
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
  }

  /**  Whichever way we are facing is now considered forward.
   * Now informs odometry of this; the wait is for thr gyro
   * to finish processing its command.
   */
  void zeroYaw() {
    gyro.zeroYaw();
    new WaitCommand(.02)
      .andThen(() -> odometry.resetRotation(Rotation2d.kZero), this)
      .schedule();
  }

  // Resets the translation of relative turn encoders to match the absolute
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {
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

/** arrange wheels in a position that resists motion in all directions.
 * the implemention sets each wheel driving very slowly away from center -- 
 * 1 robot diagonal in 256 seconds -- it should probably be slower
 */
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

/** used for calibration: each drive motor goes full speed */
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
    SmartDashboard.putNumber("gyro angle", gyro.getRotation2d().getRotations());
  }
  /** base future odometry at {@code currentPose} --
   * ignores orientation
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

  /** export odometry method */
  void visualOdometryUpdate(Pose2d newPose2d, double timestamp) {
    odometry.addVisionMeasurement(newPose2d, timestamp);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // 1. Get the current state (velocity & angle) from each module
    SwerveModuleState fl = frontLeft.getState();
    SwerveModuleState fr = frontRight.getState();
    SwerveModuleState bl = backLeft.getState();
    SwerveModuleState br = backRight.getState();

    // 2. Use your kinematics object to convert them to robot-relative speeds
    // Replace 'm_kinematics' with whatever your SwerveDriveKinematics variable is named
    return kinematics.toChassisSpeeds(fl, fr, bl, br);
}
}
