// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DrvConst;
import frc.robot.Constants.ShootConstants;

public class Robot extends TimedRobot {
  private final XboxController drive_controller = new XboxController(0);
  private final XboxController opController= new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final transferShooter actualname = new transferShooter();
  private final Feeder feeder = new Feeder();

  {
    new OurTrajectories();
    SmartDashboard.putNumber("landmark time", OurTrajectories.landmarks.get(0).state.timeSeconds);
    SmartDashboard.putNumber("landmark time1", OurTrajectories.landmarks.get(1).state.timeSeconds);
    SmartDashboard.putNumber("circle time", OurTrajectories.circleTrajectory.getTotalTimeSeconds());
  }
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  final private TrajectoryTranslator ctrlr = new TrajectoryTranslator(); 

  class TrajectoryWrap {
    Trajectory currentTrajectory;
    private final Timer timer = new Timer();
    boolean done;

    Trajectory.State getSample() {
          double timercheck = timer.get();
    Trajectory.State sample = null;
    if(currentTrajectory != null)
      {
        done = timercheck >= currentTrajectory.getTotalTimeSeconds();
        sample = currentTrajectory.sample(timercheck);
      }
    return sample;
    }
    void set(Trajectory trajectory) {
        currentTrajectory = trajectory;
        m_swerve.resetOdometry(currentTrajectory.getInitialPose());
        timer.restart();
        done = false;
      }
  
void stop() {
    currentTrajectory = null;
    done = true;
    }
  }

TrajectoryWrap trajectoryWrap;

  public class AttitudeWrap {
    AttitudePlan current;
    Timer timer = new Timer();
    AttitudePlan.State report() {
      return (current != null) ? current.report(timer.get()) : AttitudePlan.State.kZero;
    }    
  }
  AttitudeWrap currentAttitudePlan;

  void runTrajectory() { //  its use of odometry is still crude. TODO: less crude
    var sample = trajectoryWrap.getSample();
    var attitude = currentAttitudePlan.report();

    var desiredSpds = ctrlr.calculate(m_swerve.reportOdometry(), sample, attitude);

        SmartDashboard.putNumber("traj x pos", sample.poseMeters.getX());
        SmartDashboard.putNumber("traj x spd", sample.velocityMetersPerSecond);
        SmartDashboard.putNumber("calc x spd", desiredSpds.vxMetersPerSecond);
        
    m_swerve.drive(desiredSpds.vxMetersPerSecond, desiredSpds.vyMetersPerSecond, desiredSpds.omegaRadiansPerSecond, true);
    
    }
    
  // Report swerve drive data
  {addPeriodic(m_swerve::report, .25);}
  {addPeriodic(() -> SmartDashboard.putBoolean("Hood ready?", actualname.hoodReady()), .25,.125);}

  Command setHoodCommand(double level) {
    return Commands.waitSeconds(1).andThen(() -> actualname.adjustHood(level)) .andThen(Commands.waitUntil(actualname::hoodReady));
  }
  @Override
  public void autonomousInit() {
    //TODO: Command based:
    setHoodCommand(.77)
      .andThen(setHoodCommand(.05), 
        setHoodCommand((ShootConstants.kMaxPosition + ShootConstants.kMinPosition) / 2)).schedule();
    trajectoryWrap.set(OurTrajectories.circleTrajectory);
  }
  @Override
  public void autonomousPeriodic() {
    //TODO: Command based:
    CommandScheduler.getInstance().run();
    runTrajectory();
     m_swerve.updateOdometry();
  }

  static boolean useField = true, useVelCtrl = false;

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("vision target found", LimelightHelpers.getTV("limelight-a"));
    SmartDashboard.putBoolean("april tag found", LimelightHelpers.getTV("limelight-b"));
    driveWithJoystick(useField);
    m_swerve.updateOdometry();
    // if (drive_controller.getLeftStickButtonPressed()) actualname.shoot(true);
    if(opController.getAButtonPressed()) feeder.run(true);
    if(opController.getAButtonReleased()) feeder.run(false);

    //if(drive_controller.getBButton()){System.out.println("Hello world");}
    
  }

  private boolean firstTimeDisabled = true;

  @Override
  public void disabledInit() {
    if (firstTimeDisabled)
    {
      firstTimeDisabled = false;
      //m_swerve.zeroYaw();//TODO: check if need
      showFieldCtr();
    }
    LimelightHelpers.setLEDMode_ForceOff("limelight-b");
    m_swerve.setRelOffset();
  }

  private double maxSpeed = DrvConst.kMaxSpeed,
                maxAngularSpeed = DrvConst.kMaxAngularSpeed;

  void showMaxSpeeds() {
    SmartDashboard.putNumber("Max. linear speed", maxSpeed);
    SmartDashboard.putNumber("Max. angular speed", maxAngularSpeed);
  }

  void setMaxSpeed(double speed) {
    maxSpeed = speed;
  }
  
  void setMaxAngularSpeed(double speed) {
    maxAngularSpeed = speed;
  }
  
  private void showFieldCtr() {
    SmartDashboard.putBoolean("Field Centered", useField);
  }
  private void driveWithJoystick(boolean fieldRelative) {
    if(drive_controller.getLeftBumperButtonPressed()) m_swerve.setXPosture();
    if(drive_controller.getAButtonPressed()) m_swerve.zeroYaw(); /* useVelCtrl ^= true; */
    if(drive_controller.getBButtonPressed()) {
      useField ^= true;
      showFieldCtr();
    }
    if(drive_controller.getXButton()) {
      m_swerve.fullSpeed();
      return;
    }
    if(drive_controller.getYButtonPressed()) {
      setMaxSpeed(DrvConst.kMaxSpeed/4);
      setMaxAngularSpeed(DrvConst.kMaxAngularSpeed/4);
    }
    if(drive_controller.getYButtonReleased()) {
      setMaxSpeed(DrvConst.kMaxSpeed);
      setMaxAngularSpeed(DrvConst.kMaxAngularSpeed);
    }
   /*  if(drive_controller.getYButton()) {
      m_swerve.drive(1, 0, 0, false);
      return;
    }
    */ // DONE: have max speed modifiable
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        - m_xspeedLimiter.calculate(MathUtil.applyDeadband(drive_controller.getLeftY(), Constants.deadBand))
            * maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        - m_yspeedLimiter.calculate(MathUtil.applyDeadband(drive_controller.getLeftX(), Constants.deadBand))
            * maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        - m_rotLimiter.calculate(MathUtil.applyDeadband(drive_controller.getRightX(), Constants.deadBand))
            * maxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
