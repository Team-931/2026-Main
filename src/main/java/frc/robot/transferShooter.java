package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConst;
import frc.robot.Constants.ShootConstants;


public class transferShooter extends SubsystemBase { 

    final TalonFX feeder_motor = new TalonFX(FeederConst.motorID);
    /* The following bracketed code is run at set-up */
    {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    /* If the motor gets installed backward change this */
                    .withInverted(InvertedValue.Clockwise_Positive)
                    /* Don't brake when set at zero */
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                /* Keep the motors from burning out if they jam */
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit((120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit((50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                /* for velocity control if we need it */
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(Constants.nominalVoltage / Constants.krakenFreeSpeed) // 12 volts when requesting max RPS
            );
        
        feeder_motor.getConfigurator().apply(config);
    }

    Servo leftServo=new Servo(ShootConstants.leftServoID), rightServo=new Servo(ShootConstants.rightServoID);
    {
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        //adjustHood((ShootConstants.kMaxPosition + ShootConstants.kMinPosition) / 2);
    }

    TalonFX shooterLeft=new TalonFX(ShootConstants.leftShooterID), shooterMid=new TalonFX(ShootConstants.midShootID), shooterRight=new TalonFX(ShootConstants.RightShootID), 
    transfer=new TalonFX(ShootConstants.transferMotorID);
    {
        configureMotor(shooterRight, InvertedValue.Clockwise_Positive);
        configureMotor(shooterLeft, InvertedValue.CounterClockwise_Positive);
        configureMotor(shooterMid, InvertedValue.CounterClockwise_Positive);
    }
    Follower followRight = new Follower(ShootConstants.RightShootID, MotorAlignmentValue.Opposed);
    VelocityVoltage velocityRequest = new VelocityVoltage(0);

    /**  */
    //TODO orientation & performance activities
    void shoot_with_voltage(boolean on){
        shooterRight.setVoltage (on ? Constants.nominalVoltage * ShootConstants.launch_speed : 0);
    }

    /** @param velocity double in RPS
     * PIH: I was wrong and WCP confused me.
     * They give RPM TalonFX uses RPS.
     * 80 RPS is probably top speed.
     * 2600 RPM / 60 ~ 43.3 RPS
     * @see VelocityVoltage#withVelocity
     */
    double target_velocity = 0;

    void shoot_with_velocity(double velocity){
        target_velocity = velocity;
        shooterRight.setControl(velocityRequest.withVelocity(target_velocity));
        shooterLeft.setControl(velocityRequest);
        shooterMid.setControl(velocityRequest);
    }

    boolean get_shooter_ready(double v_tollerance){
        return Math.abs(shooterRight.getVelocity().getValueAsDouble()-target_velocity) < v_tollerance;
    }

    //this is how we will automatically find shooting range. Need to get distance from limelight first.
    InterpolatingDoubleTreeMap shooterHoodMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();
    {
        populate_interpolate_maps();
    }

    public void populate_interpolate_maps(){
        shooterHoodMap.put(1.2,0.2);
        shooterVelocityMap.put(1.2,52.0);

        shooterHoodMap.put(3.17,0.5);
        shooterVelocityMap.put(3.17,60.0);

        shooterHoodMap.put(4.6,0.7);
        shooterVelocityMap.put(4.6,65.0);
    }
    
    public rangefinderResults rangefind(double distance){
        return new rangefinderResults(
            shooterHoodMap.get(distance),
            shooterVelocityMap.get(distance)
        );
    }

    public class rangefinderResults{
        double hood_angle;
        double shooter_velocity;

        public rangefinderResults(double new_hood_angle,double new_shooter_velocity){
            this.hood_angle = new_hood_angle;
            this.shooter_velocity = new_shooter_velocity;
        }
    }

    void setTransfer(boolean on, boolean reverse) {
        double power = (on ? ShootConstants.transferPower : 0)*(reverse ? -1 : 1);
        transfer.set(power);
        feeder_motor.set(power);
    }

    Timer hoodTimer = new Timer();
    double hoodReadyTime;
    double hoodDirection;
    double hoodLastSet; 

    //TODO check if the requested position is too far
    void adjustHood(double out) {
        out = MathUtil.clamp(out, ShootConstants.kMinPosition, ShootConstants.kMaxPosition);
        rightServo.setPosition(out); leftServo.setPosition(out);
        double x
            = (hoodTimer.isRunning() && (x = hoodReadyTime - hoodTimer.get()) > 0) ?
            x *= hoodDirection : 0;
        hoodTimer.restart();
        hoodReadyTime = x + (out - hoodLastSet) * ShootConstants.hoodFullLengthTime;
        hoodDirection = Math.signum(hoodReadyTime);
        hoodReadyTime = Math.abs(hoodReadyTime);
        hoodLastSet = out;
    }

    boolean hoodReady() {
        return !hoodTimer.isRunning() || hoodTimer.get() >= hoodReadyTime;
    }

    // Copied from West Coast Products
    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(0)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(120)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(Constants.nominalVoltage / Constants.krakenFreeSpeed) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }
}
