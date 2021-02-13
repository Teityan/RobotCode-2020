package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subClass.Const;
import frc.robot.subClass.OriginalTimer;
import frc.robot.subClass.State;

public class PanelRotationMode {

    //ports n such (>const?)
    final static I2C.Port i2cPort = I2C.Port.kOnboard;
    final static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    ColorCode colorOutput = ColorCode.inRange;
    Servo colorSensorServo;


    OriginalTimer panelRotatingTimer;
    ColorCode preColor;
    boolean is_panelRotating = false;
    boolean is_PanelColorHasChanged;

    PanelRotationMode(Servo servo) {
        this.colorSensorServo = servo;
        panelRotatingTimer = new OriginalTimer(0.3,
            (__) -> {
                //0.3s経つまで変わらなければ回ってない
                is_panelRotating = false;
                //色が変われば回り始めたかも
                if (is_PanelColorHasChanged) {
                    panelRotatingTimer.reset();
                }
                return OriginalTimer.createArgs();
            }, 
            (__) -> {
                if (is_PanelColorHasChanged) {
                    //0.3s以内に変われば回ってる、タイマーリセット
                    is_panelRotating = true;
                    panelRotatingTimer.reset();
                }
                //0.3s以内に変わらないなら回っているかどうかわからないので放置

                return OriginalTimer.createArgs();
            });
    }

    public void changeState(State state) {

        is_PanelColorHasChanged = preColor != DetectedColor();
        preColor = DetectedColor();
        System.out.println("PanelColor:" + preColor);

        panelRotatingTimer.run();

        if (is_panelRotating) {
            state.driveState = State.DriveState.kSuperLow;
            System.out.println("panelRotating!");
        } else {
            state.driveState = State.DriveState.kMiddleLow;
        }

        switch (state.panelState) {
            case p_ManualRot:
                state.shooterState = State.ShooterState.kManual;
                state.shooterLeftSpeed = state.shooterRightSpeed = state.panelManualSpeed;
                 panelRotatingTimer.reset();
                break;

            //色合わせ　青<->赤、黄<->緑
            case p_toBlue:
                AlignPanelTo(ColorCode.red, state);
                break;

            case p_toYellow:
                AlignPanelTo(ColorCode.green, state);
                break;

            case p_toRed:
                AlignPanelTo(ColorCode.blue, state);
                break;

            case p_toGreen:
                AlignPanelTo(ColorCode.yellow, state);
                break;

            case p_DoNothing:
                panelRotatingTimer.reset();
                state.driveState = State.DriveState.kLow;
                break;
        }


    }

    //DetectedColor(ロボット側のカラーセンサーの目標値　青<->赤、黄<->緑)　で呼び出す
    private ColorCode DetectedColor() {
        int p = m_colorSensor.getProximity();
        Color detectedColor = m_colorSensor.getColor();
        double r = detectedColor.red;
        double g = detectedColor.green;
        double b = detectedColor.blue;
        if (p < 80) {
            return ColorCode.outOfRange;
        }
        if ((0.2 <= r && r < 0.4) && (0.45 <= g) && (b < 0.2)) {
            return ColorCode.yellow;
        }
        if ((0.3 <= r) && (0.2 <= g && g < 0.48) && (b < 0.3)) {
            return ColorCode.red;
        }
        if ((r < 0.3) && (0.4 <= g) && (0.2 <= b && b < 0.27)) {
            return ColorCode.green;
        }
        if ((r < 0.25) && (0.4 <= g) && (0.27 <= b)) {
            return ColorCode.blue;
        }
        return ColorCode.inRange;
    }

    private void AlignPanelTo(ColorCode c, State state) {
        extendServo();

        ColorCode sc = ColorCode.outOfRange;

        if (c == ColorCode.yellow) {
            sc = ColorCode.blue;
        }
        if (c == ColorCode.red) {
            sc = ColorCode.yellow;
        }
        if (c == ColorCode.green) {
            sc = ColorCode.red;
        }
        if (c == ColorCode.blue) {
            sc = ColorCode.green;
        }

        if (DetectedColor() == c) {
            state.shooterState = State.ShooterState.kManual;
            state.shooterLeftSpeed = state.shooterRightSpeed = 0;
        } else if (DetectedColor() == sc) {
            state.shooterState = State.ShooterState.kManual;
            state.shooterLeftSpeed = state.shooterRightSpeed = Const.shooterPanelSlowAutoSpeed;
        } else {
            state.shooterState = State.ShooterState.kManual;
            state.shooterLeftSpeed = state.shooterRightSpeed = Const.shooterPanelAutoSpeed;
        }

    }

    public void extendServo() {
        colorSensorServo.setAngle(180);
    }

    public void contractServo() {
        colorSensorServo.setAngle(0);
    }

    public enum ColorCode {
        yellow,
        red,
        green,
        blue,
        inRange,
        outOfRange
    }

}
