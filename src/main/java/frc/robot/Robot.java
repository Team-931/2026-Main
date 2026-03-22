// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d; //not using it now
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Climber.Position;
import frc.robot.Constants.ButtonBoard;
import frc.robot.Constants.DrvConst;
import frc.robot.Constants.ShootConstants;

public class Robot extends TimedRobot {
  private final XboxController drive_controller = new XboxController(0);
  private final GenericHID opController = new XboxController(1);
  private final transferShooter shooter = new transferShooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Drivetrain m_swerve = new Drivetrain();

  //create pathfollower commands
  {
    //NamedCommands ONLY supports runonce commands, so you need this goofy stack for it to work without event triggers.

    NamedCommands.registerCommand("intake", Commands.runOnce(()->{intake.intakeCommand().schedule();}));
    NamedCommands.registerCommand("outtake", Commands.runOnce(()->{intake.outtakeCommand().schedule();}));
    NamedCommands.registerCommand("agitate", Commands.runOnce(()->{intake.agitateCommand().schedule();}));
    NamedCommands.registerCommand("stowed", Commands.runOnce(()->{intake.stowedCommand(true).schedule();}));
    NamedCommands.registerCommand("cancelIntake", Commands.runOnce(()->{intake.cancelCommand().schedule();}));
  }

// Generate trajectories, and their landmarks, before game starts.
/*   {
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
 */
// Temporary version: combining a hood movement with a wait for the hood to catch up.
  Command setHoodCommand(double level) {
    return Commands.waitSeconds(1).andThen(() -> shooter.adjustHood(level)) .andThen(Commands.waitUntil(shooter::hoodReady));
  }
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  //final private TrajectoryTranslator ctrlr = new TrajectoryTranslator(); 
/* 
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
 */
//TrajectoryWrap trajectoryWrap = new TrajectoryWrap();

double distance_to_goal;
Rotation2d angle_to_goal;
Rotation2d angle_of_robot_from_ll;
boolean limelight_pose_valid;
/* 
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
 */    
  //TODO: make a team set call like I did in ftc that is called at init.
    
  public Alliance currentAlliance;

  Pose2d hub_pose = new Pose2d(4.6,4.0,Rotation2d.kZero); //to prevent throwing nulls
  
  Pose2d feild_center_pose = new Pose2d(8.270500,4.034500,Rotation2d.kZero);  

  public void set_allience_constants(){
    //get what allience we are from the driver station and store it
    currentAlliance = DriverStation.getAlliance().get();
  }

  // Report swerve drive data
  {addPeriodic(m_swerve::report, .25);}
  {addPeriodic(() -> SmartDashboard.putBoolean("Hood ready?", shooter.hoodReady()), .25,.125);}
  //{addPeriodic(() -> field.setRobotPose(m_swerve.reportOdometry()), 0.125);}
  {addPeriodic(() -> {
                      m_swerve.updateOdometry();

                      SmartDashboard.putBoolean("lime-a vision target found", LimelightHelpers.getTV("limelight-a"));
                      SmartDashboard.putBoolean("lime-b april tag found", LimelightHelpers.getTV("limelight-b"));

                      Rotation2d heading_from_swerve = m_swerve.reportOdometry().getRotation();

                      LimelightHelpers.SetRobotOrientation("limelight-a", heading_from_swerve.getDegrees(), 0, 0, 0, 0, 0);
                      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
                      
                      limelight_pose_valid = LimelightHelpers.validPoseEstimate(mt2);

                      //This used to cause crashes before checking if valid pose before doing.
                      if (limelight_pose_valid){
                        SmartDashboard.putNumber("ambiguity",mt2.rawFiducials[0].ambiguity);
                      }
                      if (mt2.pose == feild_center_pose){
                        limelight_pose_valid = false;
                      }
                      if (mt2.tagCount == 0) {
                        limelight_pose_valid = false;
                      }
                      //TODO: do a check for if we are turning quickly and redject updates

                      SmartDashboard.putNumber("heading_from_swerve", heading_from_swerve.getDegrees());
                      
                      if (limelight_pose_valid){
                        Pose2d ll_pose = mt2.pose;
                        //TODO: This code causes the heading to spin constantly - it's wrong. need to fix it before implementing.

                        Pose2d rotationless_pose = new Pose2d(ll_pose.getTranslation(),m_swerve.reportOdometry().getRotation());

                        m_swerve.visualOdometryUpdate(rotationless_pose, mt2.timestampSeconds);

                        SmartDashboard.putNumber("ll_b pose x", ll_pose.getX());
                        SmartDashboard.putNumber("ll_b pose y", ll_pose.getY());
                        SmartDashboard.putNumber("ll_b pose orientation degrees", ll_pose.getRotation().getDegrees());

                        distance_to_goal = ll_pose.getTranslation().getDistance(hub_pose.getTranslation());
                        SmartDashboard.putNumber("distance_to_goal", distance_to_goal);

                        //TODO: why does this go to 0 when facing the goal? since it's just translational components it should not change when we rotate.
                        //angle_to_goal = hub_pose.minus(ll_pose).getTranslation().getAngle(); //This gives the diverence between the current angle and goal angle
                        angle_to_goal = hub_pose.getTranslation().minus(ll_pose.getTranslation()).getAngle(); //This should be global angle reguardless of robot orientation
                        angle_of_robot_from_ll = ll_pose.getRotation();

                        SmartDashboard.putNumber("angle_to_goal", angle_to_goal.getDegrees());
                        SmartDashboard.putNumber("angle_of_robot_from_ll", angle_of_robot_from_ll.getDegrees());
                      }
                      
                      /* TODO: bellow is old code that does not work. likley issue is that limelight is returning a 0,0,0 pose instead of null.
                      //limelight localisation
                      //https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization

                      var pose = LimelightHelpers.getBotPose2d("limelght-b");
                      if (pose != null) m_swerve.visualOdometryUpdate(pose, Timer.getFPGATimestamp());
 */                      }
            , kDefaultPeriod);}
  

  static boolean useField = true, useVelCtrl = false;

  double shooter_velocity = 70;

  {
    SmartDashboard.putNumber("shooter_velocity",shooter_velocity);
  }
  
  double long_hood_distance = 1;
  {
    SmartDashboard.putNumber("long_hood_distance",long_hood_distance);
  }

  Command current_rangefind_command = rangeFind();

  Timer time_since_ll_target = new Timer();

  public Command rangeFind() {
    return Commands.startRun(
      () -> {
        time_since_ll_target.start();
      },
      () ->{
          // shooter.adjustHood(ShootConstants.kMinPosition); //.77 is the mechanical limit

        if (limelight_pose_valid){
            //use recorded data to guess what hood angle and velocity to use
            transferShooter.rangefinderResults results = shooter.rangefind(distance_to_goal);
            shooter.adjustHood(ShootConstants.kMaxPosition*results.hood_angle); //.77 is the mechanical limit
            shooter_velocity = results.shooter_velocity;
            shooter.target_velocity = shooter_velocity;

            SmartDashboard.putNumber("auto_shooter_velocity",results.shooter_velocity);
            SmartDashboard.putNumber("auto_hood_angle",results.hood_angle);
            time_since_ll_target.reset();
        } else if (time_since_ll_target.hasElapsed(1)){
            //what to do if limelight broke or can't see etc
            transferShooter.rangefinderResults results = shooter.rangefind(2);
            shooter.adjustHood(ShootConstants.kMaxPosition*results.hood_angle); //.77 is the mechanical limit
            shooter_velocity = results.shooter_velocity;
            shooter.target_velocity = shooter_velocity;
        }
      }
    );
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(useField);
    m_swerve.updateOdometry();
    CommandScheduler.getInstance().run();
    // Temporary testing


//transfershooter related things

    if(opController.getRawButtonPressed(ButtonBoard.Shoot)) {
      intake.agitateCommand().schedule();
      shooter.target_velocity = shooter_velocity;
      shooter.launchCommand().schedule();
      //force feild centric when shooting
      useField = true;
      showFieldCtr();
    }
    if(opController.getRawButton(ButtonBoard.Shoot)){
      //43.3 - Juggling speed. Doesn't shoot far.
      //60 - untested
      if (shooter.get_shooter_ready(3)){
        shooter.setTransfer(true,false);
      }
    }
    if(opController.getRawButtonReleased(ButtonBoard.Shoot)) {
      shooter.cancelCommand().schedule();
      intake.cancelCommand().schedule();
    }


    if(opController.getRawButtonPressed(ButtonBoard.FeederReverse)){
      shooter.setTransfer(true,true);
    }
    if(opController.getRawButtonReleased(ButtonBoard.FeederReverse)){
      shooter.setTransfer(false,false);
    }
    //hood stuff


    //TODO: figure out if this is the behavior we want - only rangefinding when short is pressed

    //currently this serves as my "automatic shot", not neccecarily short.
    if(opController.getRawButtonPressed(ButtonBoard.HoodShort)){
      current_rangefind_command.schedule();
    }
    //currently this serves as my "automatic shot", not neccecarily short.
    if(opController.getRawButtonReleased(ButtonBoard.HoodShort)){
      current_rangefind_command.cancel();
    }

    //currently this serves as my "manual shot", not neccecarily long.
    if(opController.getRawButton(ButtonBoard.HoodLong)){
      long_hood_distance = SmartDashboard.getNumber("long_hood_distance", kDefaultPeriod);
      shooter.adjustHood(ShootConstants.kMaxPosition*long_hood_distance); //.77 is the mechanical limit

      //TODO -- kDefaultPeriod is almost certainly not the right default for shooter_velocity
      shooter_velocity = SmartDashboard.getNumber("shooter_velocity", kDefaultPeriod);
    }


    /** @see Climber#java */
    if(opController.getRawButtonPressed(ButtonBoard.ClimberUp))
        climber.set(Position.HANGING);
    if(opController.getRawButtonPressed(ButtonBoard.ClimberDown))
        climber.set(Position.HUNG);



    //intake related stuff

    if(opController.getRawButtonPressed(ButtonBoard.IntakeUp))
      intake.stowedCommand(true).schedule();

    if(opController.getRawButtonPressed(ButtonBoard.IntakeDown))
      intake.stowedCommand(false).schedule();

    if(opController.getRawButtonPressed(ButtonBoard.FuelIn))
      intake.intakeCommand().schedule();

    if(opController.getRawButtonPressed(ButtonBoard.FuelOut))
      intake.outtakeCommand().schedule();

    if(opController.getRawButtonReleased(ButtonBoard.FuelIn)||opController.getRawButtonReleased(ButtonBoard.FuelOut))
      intake.cancelCommand().schedule();
  }

  @Override
  public void disabledExit() {
    climber.homingCommand().schedule();
    intake.homingCommand().schedule();
    set_allience_constants();
  }
  private boolean firstTimeDisabled = true;

  @Override
  public void disabledInit() {
    if (firstTimeDisabled)
    {
      firstTimeDisabled = false;
      //m_swerve.zeroYaw();//TODO: check if need
      showFieldCtr();

      PortForwarder.add(5801, "172.28.0.1", 5801);
      PortForwarder.add(5802, "172.28.0.1", 5802);
      PortForwarder.add(5803, "172.28.0.1", 5803);
      PortForwarder.add(5804, "172.28.0.1", 5804);
      PortForwarder.add(5805, "172.28.0.1", 5805);
      PortForwarder.add(5806, "172.28.0.1", 5806);
      PortForwarder.add(5807, "172.28.0.1", 5807);
      PortForwarder.add(5808, "172.28.0.1", 5808);
      PortForwarder.add(5809, "172.28.0.1", 5809);
    }
    // useful only on limelight-b
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

  PIDController turning_pid = new PIDController(3, 0, 0);

  private void driveWithJoystick(boolean fieldRelative) {
    if(drive_controller.getLeftBumperButtonPressed()) m_swerve.setXPosture();
    if(drive_controller.getAButtonPressed()) m_swerve.zeroYaw(currentAlliance == Alliance.Red); /* useVelCtrl ^= true; */

    //swap between feild centric and robot centric but only if we're not shooting
    if(drive_controller.getBButtonPressed() && !opController.getRawButton(ButtonBoard.Shoot)) {
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

    //currently this will just stop you from rotating while shooting. 
    //TODO: implement the pid so shooting causes the robot to target the goal
    final var rot = (
      opController.getRawButton(ButtonBoard.Shoot) ?
      //PID for hitting a target position - not done
        turning_pid.calculate(
            m_swerve.reportOdometry().getRotation().getRadians(), angle_to_goal.getRadians())
      :
      //gamepad related tuning
      - m_rotLimiter.calculate(MathUtil.applyDeadband(drive_controller.getRightX(), Constants.deadBand))*maxAngularSpeed
    );
        

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  //auto stuff

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
  public void autonomousInit() {
    autoCommand = autoChooser.getSelected();
    if(autoCommand != null) autoCommand.schedule();
    //TODO: Command based:
    //intake.homingCommand().schedule(); //moved to disabledExit
    //set_allience_constants();
  }
  @Override
  public void autonomousExit() {
    if(autoCommand != null) autoCommand.cancel();
  }

  @Override
  public void autonomousPeriodic() {
    //TODO: Command based:
    CommandScheduler.getInstance().run();
    //runTrajectory();
  }
}
