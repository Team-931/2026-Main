package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.FeederConst;

class Feeder {
    final TalonFX motor = new TalonFX(FeederConst.motorID);
    {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit((120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit((50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(Constants.nominalVoltage / Constants.krakenFreeSpeed) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }
    void run(boolean on) {
        if(on)
            motor.set(FeederConst.runPower) ; 
        else motor.set(0);
    }
}
