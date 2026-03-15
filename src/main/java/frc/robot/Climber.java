package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class Climber extends SubsystemBase {
    /** Runs climber to a certain position.
     * If rots is less than previous call
     * it will rewind partially
     * @param rots how many output rotations 
     * should the climber be past fully wound,
     * <p>software prevents exceeding upper and
     * lower limits.
     */
    void unwind(double rots) {
        motor.setControl(extensionRequest.withPosition(rots));
    }

    Command rewindCommand() {
        return runOnce(this::rewindInit) . andThen(Commands.waitUntil(this::rewindFinished)) . andThen(runOnce(this::afterRewind));
    }
    /** starts a slow rewind which ignores the limit */
    void rewindInit() {
        motor.setControl(rewindRequest);
    }
    /** when a rewind meets resistance
     * @return true when meets resistance
     */
    boolean rewindFinished() {
        return motor.getSupplyCurrent(true).getValueAsDouble() > Constants.Climber.rewoundAmps;
    }
    /** sets current motor position as the lower limit */
    void afterRewind() {
        motor.setPosition(0);
    }
    /** holds the motion plan */
    final MotionMagicVoltage extensionRequest = 
            new MotionMagicVoltage(0),
                            liftRequest = 
            new MotionMagicVoltage(Constants.Climber.climbedUnwind)
                .withFeedForward(Constants.Climber.feedForwardVoltage);
    final VoltageOut rewindRequest = 
        new VoltageOut(-Constants.Climber.rewindVoltage)
            .withIgnoreSoftwareLimits(true);
    // One motor
    private TalonFX motor = new TalonFX(Constants.Climber.motorID);
    // Configured for encoder values in inches, fast acceleration to cruise speed
    {
            final TalonFXConfiguration config = new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withReverseSoftLimitThreshold(0)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Constants.Climber.maxUnwind)
                    .withForwardSoftLimitEnable(true)   
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit((20))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit((70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                   .withSensorToMechanismRatio(Constants.Climber.gearing)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Constants.krakenFreeSpeed)
                    .withMotionMagicAcceleration(Constants.Climber.accel) // TODO: check carefully
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(10)
                    .withKI(0)
                    .withKD(0)
                    .withKV(Constants.nominalVoltage / Constants.krakenFreeSpeed) // 12 volts when requesting max RPS
            );

        motor.getConfigurator().apply(config);

    }
}
