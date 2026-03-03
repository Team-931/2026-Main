package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShootConstants;


public class transferShooter { 
    Servo leftServo=new Servo(ShootConstants.leftServoID), rightServo=new Servo(ShootConstants.rightServoID);
    {
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        //adjustHood((ShootConstants.kMaxPosition + ShootConstants.kMinPosition) / 2);
    }

    TalonFX shooterLeft=new TalonFX(ShootConstants.leftShooterID), shooterMid=new TalonFX(ShootConstants.midShootID), shooterRight=new TalonFX(ShootConstants.RightShootID), 
    transfer=new TalonFX(ShootConstants.transferMotorID);
    {configureMotor(shooterLeft, InvertedValue.CounterClockwise_Positive);}
    Follower followLeft = new Follower(ShootConstants.leftShooterID, MotorAlignmentValue.Opposed);
    {
        shooterMid.setControl(followLeft); 
        shooterRight.setControl(followLeft);
    }
//TODO orientation & prefomance activities
void shoot(boolean on){
    shooterLeft.setVoltage (on ? Constants.nominalVoltage * ShootConstants.launch_speed : 0);
}

void setTransfer(boolean on) {
    transfer.set(on ? ShootConstants.transferPower : 0);
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
