package frc.robot;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.RPM;


final class Constants {
    static final double deadBand = .05;
    static final double nominalVoltage = 12; /* Volts */
    static final double krakenFreeSpeed = (6000); /* RPM */
  
    static final class DrvConst {
         static final double kMaxSpeed = 3.0, overloadSpeed = SwvModConst.freeVeloc/* or kMaxSpeed */; // meters per second
         static final double kMaxAngularSpeed = 1*Math.PI; // 0.5 rotation per second, was 0.5 rad/s before
         /** Everything specific to one wheel corner */
         static final class Setup {
            /** motor CAN IDs */
            final int driveId, turnId;
            /** known orientation of absolute encoder (when wheel is forward) */
            final double absOffset;
            /** identifying string to label reports with */
            final String name;
            /** just fill the data in */
            Setup(int drv, int trn, double offset, String name) {
                driveId = drv;
                turnId = trn;
                absOffset = offset;
                this.name = name;
            }
         }

         static final double baseOffset = .1875; // 67.5 degrees or 3/16 circle
         /** each {@link absOffet} is 1/4 circle rotated from the adjacent one.*/
         static Setup frontLeft = new Setup(3, 5, baseOffset + .75, "FL"),
                     frontRight = new Setup(9, 6, baseOffset, "FR"),
                     backLeft = new Setup(2, 4, baseOffset + .5, "BL"),
                     backRight = new Setup(8, 7, baseOffset + .25, "BR");
        
         static final double halfWidth = Units.inchesToMeters(10.75), halfLength = Units.inchesToMeters(11.5);
         static final Translation2d frontLeftLocation = new Translation2d(halfWidth, halfLength); // unit: meters; x is forward dist from center, y is leftward
         static final Translation2d frontRightLocation = new Translation2d(halfWidth, -halfLength);
         static final Translation2d backLeftLocation = new Translation2d(-halfWidth, halfLength);
         static final Translation2d backRightLocation = new Translation2d(-halfWidth, -halfLength);
         
         static final Rotation2d ClockW90 = new Rotation2d(0, 1);
         static final Translation2d frontLeftClW = frontLeftLocation.rotateBy(ClockW90); 
         static final Translation2d frontRightClW = frontRightLocation.rotateBy(ClockW90);
         static final Translation2d backLeftClW = backLeftLocation.rotateBy(ClockW90);
         static final Translation2d backRightClW = backRightLocation.rotateBy(ClockW90);
         
         static final double driveRadius = 
                 MyMath.max(
                    frontLeftLocation.getNorm(), frontRightLocation.getNorm(), 
                    backLeftLocation.getNorm(), backRightLocation.getNorm()
                    );
                 
        /**  how much to correct position errors during a Trajectory:
         * units of proportion / sec
         */
         static final double traj_kP = 2;
         static final double orientationttitudeP = 0;

    }
    static final class SwvModConst {
        // factors to translate encoder readings into useful units
        static final double kWheelRadius = .05931 / 2; //meter //diameter: 2.335 in, 59.31 mm
        static final double freeVeloc = 4.63; // meters / sec, full power, no load
        static final int turnGearing = 28, driveGearing = 4;
        static final double driveConversion = 2 * Math.PI * kWheelRadius / driveGearing, // motor rotations to output meters
        // TODO Decide whether turn unit should be radians, or rotations as currently.
                            turnConversion = 1. / turnGearing;
        // Tuning for the contron loops
        final static ClosedLoopSlot posSlot = ClosedLoopSlot.kSlot0;
        final static double posP = .5;
        final static ClosedLoopSlot velSlot = ClosedLoopSlot.kSlot1;
        final static double velP = .0001;
         //TODO: tune better
        static final double DrvFF = 1 / freeVeloc; // Officially Volt /(m/s), conjectured: proportional output / (m/s)
        static final double velI = 0.001, velIZone = .05;
        static final double turnI = 0.003 * posP, turnIZone = 1. / 64;
        // what speed can be ignored for wheel orientation
        static final double minSpd = .001, // 1.0 mm / s
                            minSpdSq = minSpd*minSpd;
    }
    static final class ShootConstants {
        // percent of full power (or velocity if using velocity control)
        // for shooters when turned on
        static final double launch_speed = 1;
        // percent of full power (or velocity if using velocity control)
        // for tranfer when turned on
        static final double transferPower = .5;
        // Hood servo constants:
        // speed capacity
        static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
        // all the way in (in proportion to full length) leaves 1 mm for error 
        static final double kMinPosition = 0.01;
        // as far out as the hood can go (in proportion to full length)
        static final double kMaxPosition = 0.77;
        // we plan to allow 1 mm for error
        static final double kPositionTolerance = 0.01;
        // potential full extension of servos, if the hood geometry permitted
        static final Distance kServoLength = Millimeters.of(100);
        /** Time for hood Servos to run their full length */
        static final double hoodFullLengthTime = kServoLength.div(kMaxServoSpeed).in(Seconds);
        // CAN Id.s for TalonFX need not be distinct from those for SparkMax
        // but must differ from each other: 
        // check if they match the motors' configuration.
        static final int leftShooterID = 15;
        static final int midShootID = 16;
        static final int RightShootID = 17;
        static final int transferMotorID = 14;
        // PWM Id.s match the ports they plug into
        static final int leftServoID = 1;
        static final int rightServoID = 9;
    }
    static final class FeederConst {
    
        // CAN Id.s for TalonFX need not be distinct from those for SparkMax
        // but must differ from each other: 
        // check if it matches the motor's configuration.
        static final int motorID = 13;
        // percent of full power (or velocity if using velocity control) when turned on
        static final double runPower = .5;
    }
    static final class Intake {
        // TODO: adjust all constants.
        /** motor rotations from fully closed to fully extended */
        public static final double inoutRange = 7;
        //
        public static final double inoutSpeed = krakenFreeSpeed/(inoutRange), // as fast as possible
                inoutAccel = 1; // = 1 / (time from 0 to 1) squared
        //
        public static final int inoutID = 12;
        public static final int pickupID = 11;
        public static final double pickupPower = .5;
        /** How high to go up when agitating
         * <p> [0 to 1] lower numbers mean higher position.
         */
        static final double agitateUpper = .8;
    }

    public static class IntakeConstants {
        public static final int fuelIntakeID = 11;
        public static final int outInID = 12;

    }

    public static class ClimberConstants {
        public static final int climberID = 18;
    
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

}
