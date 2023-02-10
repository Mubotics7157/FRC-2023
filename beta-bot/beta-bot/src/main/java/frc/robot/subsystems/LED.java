package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends CommandBase{
    private static LED instance = new LED();
    CANdle candle;
    CANdleConfiguration config;
    RainbowAnimation rainbowAnim;
    StrobeAnimation strobeAnim;
    LarsonAnimation larsonAnim;


    public LED(){
        candle = new CANdle(31);
        config = new CANdleConfiguration();
        candle.configFactoryDefault();
        config.stripType = LEDStripType.RGB;

        config.brightnessScalar = 0.5;
        candle.setLEDs(255, 255, 255);
        candle.clearAnimation(0);
        rainbowAnim = new RainbowAnimation(1, 0.85, 300);
        strobeAnim = new StrobeAnimation(0, 255, 0, 0, 0, 300);
        larsonAnim = new LarsonAnimation(255, 25, 0, 0, 0.5, 300, BounceMode.Back, 25);
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config);
    }

    public static LED getInstance(){
        return instance;
    }

    public void setRainbow(){
        candle.animate(rainbowAnim);
    }

    public void setStrobe(){
        candle.animate(strobeAnim);
    }

    public void setPewPew(){
        candle.animate(larsonAnim);
    }

    public void offAnim(){
        candle.clearAnimation(0);
    }

    public void setRed(){
        candle.setLEDs(255, 0, 0);
    }

    public void setGreen(){
        candle.setLEDs(0, 255, 0);
    }

    public void setBlue(){
        candle.setLEDs(0, 0, 255);
    }

    public void setYellow(){
        offAnim();
        candle.setLEDs(255, 100, 0);
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    public void setOrange(){
        offAnim();
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
        candle.setLEDs(255, 25, 0);
    }

    public void setOff(){
        offAnim();
        candle.setLEDs(0, 0, 0);
    }

    public void setPurple(){
        offAnim();
        candle.setLEDs(255, 0, 50);
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }
}