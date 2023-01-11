package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    CANdle candle;
    CANdleConfiguration config;
    RainbowAnimation rainbowAnim;


    public LED(){
        candle = new CANdle(20);
        config = new CANdleConfiguration();
        candle.configFactoryDefault();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;
        candle.setLEDs(255, 255, 255);
        rainbowAnim = new RainbowAnimation(1, 1, 50);
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config);
    }

    public void setRainbow(){
        candle.animate(rainbowAnim);
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
        candle.setLEDs(255, 255, 0);
    }

    public void setOrange(){
        candle.setLEDs(255, 91, 31);
    }

    public void setOff(){
        candle.setLEDs(0, 0, 0);
    }

    public void setPurple(){
        candle.setLEDs(230, 230, 250);
    }
}
