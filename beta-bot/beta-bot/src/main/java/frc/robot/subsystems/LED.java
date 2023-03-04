package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;


public class LED {
    private static LED instance = new LED();
    CANdle candle;
    CANdleConfiguration config;
    RainbowAnimation rainbowAnim;
    StrobeAnimation strobeAnim;
    StrobeAnimation redStrobe;
    StrobeAnimation purpleStrobe;
    StrobeAnimation yellowStrobe;
    LarsonAnimation larsonAnim;
    FireAnimation FIREEE;
    ColorFlowAnimation colorFLow;
    SingleFadeAnimation orangeFade;


    public LED(){
        candle = new CANdle(31);
        configSettings();

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

    public void strobeCurrentIntake(){
        if(Intake.getInstance().isClosed())
            candle.animate(yellowStrobe);
        else
            candle.animate(purpleStrobe);
    }

    public void setCurrentIntake(){
        if(Intake.getInstance().isClosed())
            setYellow();
        else
            setPurple();
    }

    public void setRedStrobe(){
        candle.animate(redStrobe);
    }

    public void setPurpleStrobe(){
        candle.animate(purpleStrobe);
    }

    public void setYellowStrobe(){
        candle.animate(yellowStrobe);
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
        
        //candle.animate(larsonAnim);
    }

    public void setOrangeFade(){
        
        candle.animate(orangeFade);
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

    private void configSettings(){
        config = new CANdleConfiguration();
        candle.configFactoryDefault();
        config.stripType = LEDStripType.RGB;

        config.brightnessScalar = .5;
        candle.setLEDs(255, 255, 255);
        candle.clearAnimation(0);
        rainbowAnim = new RainbowAnimation(1, 0.85, 300);
        strobeAnim = new StrobeAnimation(0, 255, 0, 0, 0, 300);
        redStrobe = new StrobeAnimation(255, 0, 0, 0, 0, 300);
        yellowStrobe = new StrobeAnimation(255, 100, 0, 0 , 0, 300);
        purpleStrobe = new StrobeAnimation(255, 0, 50, 0, 0, 0, 300);
        FIREEE = new FireAnimation(1, 1, 300, 0.5, 0.5);
        colorFLow = new ColorFlowAnimation(255, 25, 0, 0, 0.5, 250, Direction.Forward, 0);
        larsonAnim = new LarsonAnimation(255, 25, 0, 0, 0.5, 300, BounceMode.Back, 25);
        orangeFade = new SingleFadeAnimation(255, 25, 0, 0, 0.5, 300);
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config);
    }
}
