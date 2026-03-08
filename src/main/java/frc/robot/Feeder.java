package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConst;
/** Represents controls for the feeder hardware */
class Feeder extends SubsystemBase {
    /** One motor: Kraken's controller is TalonFX */
    final TalonFX motor = new TalonFX(FeederConst.motorID);
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
        
        motor.getConfigurator().apply(config);
    }
    /** On/off control
     * @param on : {@code true} turns on {@code false} turns off.
     */
    void run(boolean on) {
        if(on)
            motor.set(FeederConst.runPower) ; 
        else motor.set(0);
    }
}
