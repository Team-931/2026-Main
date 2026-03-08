package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

class Intake extends SubsystemBase {
    /** Pulls the intake in to fully folded */
    void in() {
        inoutMotor.setControl(inoutRequest.withPosition(0));
    }
    /** Lets the intake fully unfold */
    void out() {
        inoutMotor.setControl(inoutRequest.withPosition(1));
    }
    /** Moves intake up a little to shake fuel */
    void agitateUp() {
        inoutMotor.setControl(inoutRequest.withPosition(Constants.Intake.agitateUpper));
    }
    /** when agitating alternate between {@link Intake#out()} and {@link Intake#agitateUp()} */
    void agitateSwitch() {
        if(atRightHeight.refresh().getValue())  {
            double height = inoutRequest.Position;
            if(height == 1 || height < Constants.Intake.agitateUpper) 
                inoutRequest.withPosition(Constants.Intake.agitateUpper);
            else inoutRequest.withPosition(1);
            }
        inoutMotor.setControl(inoutRequest);
    }
    /** run the pick-up rollers */
    void pickup() {
        pickupMotor.setVoltage(Constants.nominalVoltage * Constants.Intake.pickupPower);
    }
    /** stop the pick-up rollers */
    void stop() {
        pickupMotor.setVoltage(0);
    }
    private MotionMagicVoltage inoutRequest = new MotionMagicVoltage(0).withSlot(0);
    private TalonFX inoutMotor =  new TalonFX(Constants.Intake.inoutID),
                    pickupMotor = new TalonFX(Constants.Intake.pickupID);
    {
        // config for pickupMotor
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(120)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLimitEnable(true)
            );
        pickupMotor.getConfigurator().apply(config);
        // Extra config for inoutMotor
        config
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(Constants.Intake.inoutRange)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                // as fast as possible
                    .withMotionMagicCruiseVelocity(Constants.Intake.inoutSpeed)
                // 
                    .withMotionMagicAcceleration(Constants.Intake.inoutAccel)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(6 * Constants.Intake.inoutRange)
                    .withKI(0)
                    .withKD(0)
                     // 12 volts when requesting max RPS
                    .withKV(12.0 / Constants.Intake.inoutSpeed)
            );
        inoutMotor.getConfigurator().apply(config);
    }
    StatusSignal<Boolean> atRightHeight = inoutMotor.getMotionMagicAtTarget();
}
