// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.DrvConst;
import frc.robot.Constants.ShootConstants;

public class Robot extends TimedRobot {
  private final XboxController drive_controller = new XboxController(0);
  private final GenericHID opController = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final transferShooter shooter = new transferShooter();
  private final Intake intake = new Intake();

// Generate trajectories, and their landmarks, before game starts.
  {
    new OurTrajectories();
    SmartDashboard.putNumber("landmark time", OurTrajectories.circlelandmarks.get(0).state.timeSeconds);
    SmartDashboard.putNumber("landmark time1", OurTrajectories.circlelandmarks.get(1).state.timeSeconds);
    SmartDashboard.putNumber("circle time", OurTrajectories.circleTrajectory.getTotalTimeSeconds());
  }
// Display trajectories, and their landmarks, before game starts.
  Field2d field = new Field2d();
  {
    SmartDashboard.putData(field);
    field.getObject("circle").setTrajectory(OurTrajectories.circleTrajectory);
    field.getObject("lmk 0").setPose(OurTrajectories.circlelandmarks.get(0).state.poseMeters);
    field.getObject("lmk 1").setPose(OurTrajectories.circlelandmarks.get(1).state.poseMeters);
    field.getObject("toBalls").setTrajectory(OurTrajectories.centerBalls);
    field.getObject("near balls").setPose(OurTrajectories.centerBall2Landmark.state.poseMeters);
  }
// Temporary version: combining a hood movement with a wait for the hood to catch up.
  Command setHoodCommand(double level) {
    return Commands.waitSeconds(1).andThen(() -> shooter.adjustHood(level)) .andThen(Commands.waitUntil(shooter::hoodReady));
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

TrajectoryWrap trajectoryWrap = new TrajectoryWrap();

// TODO: allow setting current OrientationPlan
  public class OrientationWrap {
    OrientationPlan current;
    Timer timer = new Timer();
    OrientationPlan.State report() {
      return (current != null) ? current.report(timer.get()) : null;
    }    
  }
  OrientationWrap currentOrientationPlan = new OrientationWrap();

  void runTrajectory() { //  its use of odometry is still crude. TODO: less crude
    var sample = trajectoryWrap.getSample();
    var orientation = currentOrientationPlan.report();

    var desiredSpds = ctrlr.calculate(m_swerve.reportOdometry(), sample, orientation);
      if(sample != null) {
        SmartDashboard.putNumber("traj x pos", sample.poseMeters.getX());
        SmartDashboard.putNumber("traj x spd", sample.velocityMetersPerSecond);
      }
        SmartDashboard.putNumber("calc x spd", desiredSpds.vxMetersPerSecond);
        
    m_swerve.drive(desiredSpds.vxMetersPerSecond, desiredSpds.vyMetersPerSecond, desiredSpds.omegaRadiansPerSecond, true);
    
    }
    
  // Report swerve drive data
  {addPeriodic(m_swerve::report, .25);}
  {addPeriodic(() -> SmartDashboard.putBoolean("Hood ready?", shooter.hoodReady()), .25,.125);}
  {addPeriodic(() -> field.setRobotPose(m_swerve.reportOdometry()), 0.125);}
  {addPeriodic(() -> {
                      m_swerve.updateOdometry();
/* TODO: why does this fail??
                      //limelight localisation
                      //https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization

                      var pose = LimelightHelpers.getBotPose2d("limelght-b");
                      if (pose != null) m_swerve.visualOdometryUpdate(pose, Timer.getFPGATimestamp());
 */                      }
            , kDefaultPeriod);}
  

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
  // This sets up the choices for auto
  // now only simple examples 
  {
    autoChooser.setDefaultOption("No autonomous", null);
    // autoChooser.addOption("Move hood", 
    //   setHoodCommand(.77)
    //   .andThen(setHoodCommand(.05), 
    //     setHoodCommand((ShootConstants.kMaxPosition + ShootConstants.kMinPosition) / 2)));
    // autoChooser.addOption("Circle trajectory", Commands.runOnce(() -> trajectoryWrap.set(OurTrajectories.circleTrajectory)));
    
    SmartDashboard.putData("Auto chooser", autoChooser);//TODO: add a label
  }

  private Command autoCommand;

  @Override
  public void teleopInit(){
    intake.homingCommand().schedule();
  }

  @Override
  public void autonomousInit() {
    autoCommand = autoChooser.getSelected();
    if(autoCommand != null) autoCommand.schedule();
    //TODO: Command based:
    intake.homingCommand().schedule();
  }
  @Override
  public void autonomousExit() {
    if(autoCommand != null) autoCommand.cancel();
  }

  @Override
  public void autonomousPeriodic() {
    //TODO: Command based:
    CommandScheduler.getInstance().run();
    runTrajectory();
  }

  static boolean useField = true, useVelCtrl = false;

  Command current_intake_command = intake.intakeCommand(true);

  double shooter_velocity = 30;

  {
    SmartDashboard.putNumber("shooter_velocity",shooter_velocity);
  }
  
  double long_hood_distance = 0.7;
  {
    SmartDashboard.putNumber("long_hood_distance",long_hood_distance);
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("vision target found", LimelightHelpers.getTV("limelight-a"));
    SmartDashboard.putBoolean("april tag found", LimelightHelpers.getTV("limelight-b"));

    double heading_from_swerve = m_swerve.reportOdometry().getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation("limelight", heading_from_swerve, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
    Pose2d ll_pose = mt2.pose;

    SmartDashboard.putNumber("heading_from_swerve", heading_from_swerve);

    SmartDashboard.putNumber("ll_b pose x", ll_pose.getX());
    SmartDashboard.putNumber("ll_b pose y", ll_pose.getY());
    SmartDashboard.putNumber("ll_b pose orientation degrees", ll_pose.getRotation().getDegrees());

    driveWithJoystick(useField);
    m_swerve.updateOdometry();
    CommandScheduler.getInstance().run();
    // Temporary testing


    

//button board not working at all?
    if(opController.getRawButtonPressed(ButtonBoard.Shoot)) {
      //TODO -- kDefaultPeriod is almost certainly not the right default for shooter_velocity
      shooter_velocity = SmartDashboard.getNumber("shooter_velocity", kDefaultPeriod);
      shooter.shoot_with_velocity(shooter_velocity);
      current_intake_command = intake.agitateCommand();
      current_intake_command.schedule();
    }
    if(opController.getRawButton(ButtonBoard.Shoot)){
      //43.3 - Juggling speed. Doesn't shoot far.
      //60 - untested
      if (shooter.get_shooter_ready(3)){
        shooter.setTransfer(true,false);
      }
    }
    if(opController.getRawButtonReleased(ButtonBoard.Shoot)) {
      shooter.setTransfer(false,false);
      shooter.shoot_with_velocity(0); //we don't really care what it returns
      current_intake_command.cancel();
    }
    if(opController.getRawButtonPressed(ButtonBoard.FeederReverse)){
      shooter.setTransfer(true,true);
    }
    if(opController.getRawButtonReleased(ButtonBoard.FeederReverse)){
      shooter.setTransfer(false,false);
    }

    //hood stuff!!

    if(opController.getRawButton(ButtonBoard.HoodShort)){
      shooter.adjustHood(ShootConstants.kMinPosition); //.77 is the mechanical limit
    }

    if(opController.getRawButton(ButtonBoard.HoodLong)){
      long_hood_distance = SmartDashboard.getNumber("long_hood_distance", kDefaultPeriod);
      shooter.adjustHood(ShootConstants.kMaxPosition*long_hood_distance); //.77 is the mechanical limit
    }

    if(opController.getRawButtonPressed(ButtonBoard.IntakeUp)){
      intake.stowedCommand(true).schedule();
    }
    if(opController.getRawButtonPressed(ButtonBoard.IntakeDown)){
      intake.stowedCommand(false).schedule();
    }

    if(opController.getRawButtonPressed(ButtonBoard.FuelIn)){
      current_intake_command = intake.intakeCommand(true);
      current_intake_command.schedule();
    }

    if(opController.getRawButtonPressed(ButtonBoard.FuelOut)){
      current_intake_command = intake.intakeCommand(false);
      current_intake_command.schedule();
    }

    if(opController.getRawButtonReleased(ButtonBoard.FuelIn)||opController.getRawButtonReleased(ButtonBoard.FuelOut)){
      current_intake_command.cancel();
    }
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
    if(true) { //there are better ways to call this stuff less.
      double slowdown_multiplier = 1-drive_controller.getLeftTriggerAxis()*0.75;
      setMaxSpeed(DrvConst.kMaxSpeed*slowdown_multiplier);
      setMaxAngularSpeed(DrvConst.kMaxAngularSpeed*slowdown_multiplier);
    }
   
    // DONE: have max speed modifiable
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
