package frc.robot.subClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * アームのクラス
 */
public class Arm {

    /** モーター */
    TalonSRX Motor;          
    
    /** 角度を測る半固定抵抗 */
    SensorCollection Encoder;

    /** アームセンサー */
    ArmSensor armSensor;

    public double armTargetChange = -30;
    double armDControl;
    double armOutput;
    double armTargetAngleOld;


    /** コンストラクター
     * 
     * @param ArmMotor アーム本体を回すためのモーター
     * @param ArmEncoder アームの角度をとるためのエンコーダー
     * @param armSensor 前後のリミットスイッチ
     */
    public Arm(TalonSRX ArmMotor, SensorCollection ArmEncoder, ArmSensor armSensor) {
        this.Motor = ArmMotor;
        this.Encoder = ArmEncoder;
        this.armSensor = armSensor;
    }

    /** 
     * Stateに応じた動作をさせる.
     * 
     * <p> アーム関係の動作を行う
     * @param state State
     */
    public void applyState(State state) {
        SmartDashboard.putNumber("armpoint", Encoder.getAnalogInRaw());

        switch (state.armState) {
            // 砲台の角度を基本状態に
            case k_Basic:
                armChangeBasic(state.armAngle);
                break;
            // 砲台の角度をPIDで制御
            case k_PID:
                armPIDMove(state.armSetAngle, state.armAngle);
                break;
            // 砲台の角度を微調整 正か負のみ
            case k_Adjust:
                armAdjust(state);
                break;
            // 手動操作
            case k_Manual:
                armMove(state.armMotorSpeed);
                break;
            // 維持する
            case k_Conserve:
                armStop(state.armAngle);
                //armPIDControl(state.armAngle, state.armAngle);
                //armTargetChange = state.armAngle;
                break;
            //指定した角度
            case k_ConstAng:
                armPIDControl(state.armFinalTargetAngle, state.armAngle, state.armTargetAngle);
                state.armTargetAngle = state.armTargetAngle + armTargetChange;
                //armPIDControl(state.armAngle, state.armAngle);
                break;
            //何もしない
            case k_DoNothing:
                armMove(0);
        }
    }

    /** 砲台の角度を微調整する（PID無し）正か負のみ */
    void armAdjust(State state) {
        if (armSensor.getArmFrontSensor()) {
            //角度が初期状態（-30度）
            if (state.armMotorSpeed > 0) {
                armMove(Const.ArmBasicUpSpeed);
            } else {
                armMove(0);
            }
        } else if (armSensor.getArmBackSensor()) {
            //角度が最も上（80度）
            if (state.armMotorSpeed < 0) {
                //LowSpeedで下げると下がらないので少し早め
                armMove(Const.ArmHighDownSpeed);
            } else {
                armMove(0);
            }
        } else {
            //角度が-30度～80度
            if (state.armMotorSpeed > 0) {
                System.out.println("armUp!!!");
                armMove(state.armMotorSpeed);
            } else if (state.armMotorSpeed < 0) {
                armMove(Const.ArmLowDownSpeed);
            } else {
                //念のため、入力ナシならStop
                armStop(state.armAngle);
            }

        }
    }

    /** 
     * 砲台のモーターを回す.
     * 
     * @param setSpeed 回す速さ
     */
    void armMove(double setSpeed) {
        Motor.set(ControlMode.PercentOutput, setSpeed);
        SmartDashboard.putNumber("setSpeedMove", setSpeed);
    }

    /** 
     * 重力オフセットを使う事でアームを停止させる.
     * 
     * <p> 出力の値は{@code SetFeedForward()}で決める
     * 
     * @param nowAngle 現在の角度
     * */
    void armStop(double nowAngle) {
        Motor.set(ControlMode.PercentOutput, 0,
                DemandType.ArbitraryFeedForward, setFeedForward(nowAngle));
        SmartDashboard.putNumber("Stop", nowAngle);
    }

    /** 
     * 砲台のモーターを回すPID制御.
     * 
     * <p> エンコーダー目標値は{@code SetPoint()}で決める <br>
     * <p> 重力オフセットは{@code SetFeedForward()}で決める
     * 
     * @param targetAngle 目標角度
     * @param nowAngle 現在の角度
     */
    void armPIDMove(double targetAngle, double nowAngle) {
        Motor.set(ControlMode.Position, setPoint(targetAngle),
                DemandType.ArbitraryFeedForward, setFeedForward(nowAngle));
    }

    /**
     * 砲台をゆっくり下ろす.  
     * 
     * @param nowAngle 現在角度。この角度によって入力速度を変化させる
     */   
    void armChangeBasic(double nowAngle) {
        if (!armSensor.getArmFrontSensor()) {
            // 角度下限認識スイッチが反応するまで落とす
            if (nowAngle > Const.ArmDownBorderAngle) {
                //早く落とす
                SmartDashboard.putNumber("High", nowAngle);
                armMove(Const.ArmHighDownSpeed);
            } else {
                //ゆっくり落とす
                SmartDashboard.putNumber("Low", nowAngle);
                armMove(Const.ArmLowDownSpeed);
            }
        } else {
            armMove(0);
        }
    }

    // 砲台角度制御に関する計算式

    /**  
     * 現在の砲台の角度を計算.
     * 
     * <p> (角度の最大最小差分) /（エンコーダー値の最大最小差分) * (エンコーダーの現在値最小値差分) + (角度の最小値)
     */
    public double getArmNow() {
        return Const.armAngleDifference / Const.armPointDifference *
                (Encoder.getAnalogInRaw() - Const.armMinPoint) + Const.armMinAngle;
    }

    /** 
     * 目標角度に合わせたPIDの目標値を計算.
     * 
     * <p> (角度の目標値最小値差分) *（エンコーダー値の最大最小差分) / (角度の最大最小差分) + (最小値からの差分)
     * 
     * @param targetAngle 角度の目標値
     */
    double setPoint(double targetAngle) {
        return (targetAngle - Const.armMinAngle) * Const.armPointDifference /
                Const.armAngleDifference + Const.armMinPoint;
    }

    /** 
     * 目標角度に合わせた重力オフセットを計算.
     * 
     * <p> (地面と水平な時の重力オフセット) * (cos現在角度)
     * 
     * @param nowAngle 現在の角度
     */
    public double setFeedForward(double nowAngle) {
        return Const.armMaxOffset * Math.cos(Math.toRadians(nowAngle));
    }

    /** 
     * 台形制御、PID制御
     * 
     * @param finalTargetAngle 最終的な目標角度
     * @param nowAngle 現在の角度（エンコーダからの値）
     */
    void armPIDControl(double finalTargetAngle, double nowAngle, double targetAngle) {
        armTargetAngleOld = targetAngle;
        // 台形制御
        if(Math.abs(finalTargetAngle - targetAngle) > Const.Acceleration) {
            if(finalTargetAngle - targetAngle > 0) {
                armTargetChange = Const.Acceleration;
            }
            else {
                armTargetChange = - Const.Acceleration;
            }
        }
        else {
            armTargetChange = finalTargetAngle - targetAngle;
        }
        // D制御
        armDControl = (targetAngle + armTargetChange - nowAngle) * (targetAngle - nowAngle) * 0.001;
        // 出力
        armOutput = (targetAngle + armTargetChange - nowAngle) / Const.Acceleration * Const.ArmMaxSpeed + armDControl + setFeedForward(nowAngle);
        if(targetAngle + armTargetChange - nowAngle > 0.15) {
            armOutput = Const.ArmMaxSpeed + armDControl + setFeedForward(nowAngle);
        }
        else if(targetAngle + armTargetChange - nowAngle < -0.15) {
            armOutput = -Const.ArmMaxSpeed + armDControl + setFeedForward(nowAngle);
        }
        armMove(armOutput);
        System.out.println(armOutput);
        System.out.println(nowAngle);
    }
}