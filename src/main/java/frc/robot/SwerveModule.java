// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwvModConst;
import frc.robot.Constants.DrvConst.Setup;
/**Represents a wheel and its necessary motors, sensors, and information. */
public class SwerveModule {
  /** The constants appropriate to this wheel: as provided at start-up. */
  private final Setup info;
  /** What the absolute encoder reads when wheel is pointd forward. */
  private final double absOffset;
  /** Negative of what the turning encoder reads when wheel is forward. */
  private double relOffset;
  // motors
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;
  // encoders internal to motors
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  // encoder external to turning motor, but reporting to its controller
  private final SparkAnalogSensor absoluteEncoder;
  private final AnalogEncoder analogEncoder;

  // feed-back routines run by motor controllers
  private final SparkClosedLoopController drivePIDController;
 
  private final SparkClosedLoopController turningPIDController;


  /**
   * Construct a SwerveModule from a {@link Setup} class
   */  
SwerveModule (Setup setup){
    info = setup;
    driveMotor = new SparkMax(setup.driveId, MotorType.kBrushless);
    turningMotor = new SparkMax(setup.turnId, MotorType.kBrushless);
    absOffset = setup.absOffset;

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
    absoluteEncoder = turningMotor.getAnalog();
    {// if the port number is valid, use it
      int port = setup.analogPort;
      analogEncoder =
         port >= 0 && port < 4 ? 
         new AnalogEncoder(port) : 
         null;
    }
    drivePIDController = driveMotor.getClosedLoopController();
    turningPIDController =
      turningMotor.getClosedLoopController();

    SparkMaxConfig configDrv = new SparkMaxConfig(),
                  configTrn = new SparkMaxConfig();
      // TODO: check voltage comp
    configDrv.voltageCompensation(Constants.nominalVoltage);      
    configTrn.voltageCompensation(Constants.nominalVoltage);      
    configDrv.encoder.positionConversionFactor(SwvModConst.driveConversion)       // New unit: meters
                  .velocityConversionFactor(SwvModConst.driveConversion / 60); // New unit: meters / second
    configDrv.closedLoop.p(SwvModConst.posP / SwvModConst.driveConversion, SwvModConst.posSlot)
                     .i(SwvModConst.velI, SwvModConst.velSlot) // This compensates for inaccuracy in feed-forward
                     .iZone(SwvModConst.velIZone, SwvModConst.velSlot) // When we change veloc. by more than this let feed-forward do the work
                     .feedForward.kV(SwvModConst.DrvFF, SwvModConst.velSlot); // about enough to reach the right veloc. without feed-back

    driveMotor.configure(configDrv, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configTrn.encoder.positionConversionFactor(SwvModConst.turnConversion)       // New unit: radians
                  .velocityConversionFactor(SwvModConst.turnConversion / 60); // New unit: radians / second
    configTrn.analogSensor.positionConversionFactor(1 / 3.35)       // New unit: rotations not radians
                  .velocityConversionFactor(1 / 3.35 / 60); // New unit: rotations not radians / second
    configTrn.closedLoop.p(SwvModConst.posP / SwvModConst.turnConversion, SwvModConst.posSlot) // main control for angle
                     .i(SwvModConst.turnI / SwvModConst.turnConversion, SwvModConst.posSlot) // overcome resistance at small error
                     .iZone(SwvModConst.turnIZone, SwvModConst.posSlot) // ignore .i for larger error
                     .p(SwvModConst.velP / SwvModConst.turnConversion * 60, SwvModConst.velSlot)
                     .positionWrappingEnabled(true)
                     .positionWrappingInputRange(-.25, .25) // 90 degrees clockwise equivalent to 90 degrees counterClwise
                     .feedForward.kV(0, SwvModConst.velSlot); // Todo:  check this

    turningMotor.configure(configTrn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setRelOffset();
}
/** Use the external encoder if it's valid, otherwise an attached one.
 * @return a value 0.0 to 1.0 representing the wheel's rotation from -{@link SwerveModule#absOffset}
 */
  final double getAbsPosition() {
    return analogEncoder == null ? 
      absoluteEncoder.getPosition() :
      analogEncoder.get();
  }

  /** sets conversion between absolute and relative encoders on turning motor */
   final void setRelOffset() {
    relOffset = getAbsPosition() - absOffset - turningEncoder.getPosition();
   }

   /** converts a relative encoder reading of current orientation to correct value */
  private final double turnRots() {
    return turningEncoder.getPosition() + relOffset;
  }

  /** Informs the turning motor's control loop of a target orientation. */
  final void setTurnRot(double angle) {
    turningPIDController.setSetpoint(angle - relOffset, ControlType.kPosition, SwvModConst.posSlot);
  }
  
  /** relative encoder reading of current orientation as a {@link Rotation2d} */
  private final Rotation2d turnAngle() {
    return Rotation2d.fromRotations(turnRots());
    }

  /**
   * Returns the current state of the module, combining speed and direction.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), turnAngle());
  }

  /**
   * Returns the current wheel position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), turnAngle());
  }

  /** used by {@link Drivetrain#report()} */
  void report() {
    SmartDashboard.putNumber(info.name + " angle", turnRots());
    SmartDashboard.putNumber(info.name + " speed", driveEncoder.getVelocity());
    SmartDashboard.putNumber(info.name + " abs. angle", getAbsPosition() - absOffset);
    SmartDashboard.putNumber(info.name + " angle diff", getAbsPosition() - absOffset - turnRots());
  }

  /** used by {@link Drivetrain#fullSpeed()} */
  void fullSpeed() {
    driveMotor.set(1);
  }
/*   
private boolean noLaborSaving = false;

  void doAngle360(boolean yes) {//TODO: don't need after abs encoders are in
    noLaborSaving = yes;
    double range = yes ? .5: .25;
    var config = new SparkMaxConfig(); config.closedLoop.positionWrappingInputRange(-range, range);
    turningMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private double X;//moved out of setVel only for reporting purpose

 */  //private Translation2d velGoal = new Translation2d();
 /** Carry out the wheel's intended speed and direction */
  public void setVel(Translation2d translation2d) {
    Translation2d velGoal = translation2d.rotateBy(turnAngle().unaryMinus());
    // The conditional is only useful with useVelCtrl
    double X = velGoal.getX() == 0 ? 1e-10 : velGoal.getX();//DONE: make X local again
    drivePIDController.setSetpoint(X, ControlType.kVelocity, SwvModConst.velSlot);//Done: check conversion factors

    if (Robot.useVelCtrl) {
//      not in use but possible to fix up
      turningPIDController.setSetpoint(velGoal.getY()/X / Robot.kDefaultPeriod, ControlType.kVelocity, SwvModConst.velSlot);
    }
    // Adjust wheel orientation if wheel speed is noticeable.
    else if(translation2d.getSquaredNorm() >= SwvModConst.minSpdSq) {
      double angle = Math.atan2(translation2d.getY(), translation2d.getX()) / 2 / Math.PI;
      setTurnRot(angle);
    }
  }
 
}
