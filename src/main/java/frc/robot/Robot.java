package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import frc.robot.subClass.*;

//TalonSRX&VictorSPXのライブラリー

public class Robot extends TimedRobot {

    //コントローラー
    //とりあえず、XboxController2つ
    XboxController driver, operator;

    //DriveMotor
    WPI_TalonSRX driveRightFrontMotor, driveLeftFrontMotor;
    VictorSPX driveRightBackMotor, driveLeftBackMotor;

    //ShooterMotor
    TalonSRX shooterLeftMotor, shooterRightMotor;

    //ArmMotor
    TalonSRX armMotor;

    //IntakeMotor
    VictorSPX intakeBeltFrontMotor, intakeBeltBackMotor;
    VictorSPX intakeMotor;

    //climbMotor
    TalonSRX climbMotor;
    Servo climbServo;
    TalonSRX slideMotor;

    //センサー
    DigitalInput intakeFrontSensor, intakeBackSensor;
    SensorCollection armEncoder;
    Servo colorSensorServo;
    Timer servoTimer;
    boolean servoTimerStarted;

    //カメラ
    CameraServer driveCamera, armCamera;

    //SubClass
    Drive drive;
    State state;
    Shooter shooter;
    Intake intake;
    IntakeBelt intakeBelt;
    Arm arm;
    ArmSensor armSensor;

    //モード
    PanelRotationMode panelRotationMode;
    ClimbMode climbMode;

    //Autonomous
    Timer autonomousTimer;
    String gameData;

    @Override
    public void robotInit() {

        //Intakeセンサー
        intakeFrontSensor = new DigitalInput(Const.IntakeBeltSensorFrontPort);
        intakeBackSensor = new DigitalInput(Const.IntakeBeltSensorBackPort);

        //シューターのモーター
        shooterLeftMotor = new TalonSRX(Const.shooterLeftMotor);
        shooterRightMotor = new TalonSRX(Const.shooterRightMotor);

        //アームのモーター
        armMotor = new TalonSRX(Const.armMotor);
        //アームのセンサー
        armMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        armMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        armEncoder = new SensorCollection(armMotor);

        //カラーセンサーのサーボ
        colorSensorServo = new Servo(Const.colorSensorServoPort);
        servoTimer = new Timer();

        //IntakeBelt
        intakeBeltFrontMotor = new VictorSPX(Const.intakeBeltFrontMotor);
        intakeBeltBackMotor = new VictorSPX(Const.intakeBeltBackMotor);
        intakeMotor = new VictorSPX(Const.IntakeMotorPort);

        //IntakeBeltのフォローの設定　※しない
        //intakeBeltBackMotor.follow(intakeBeltFrontMotor);

        //Climb Motor
        climbMotor = new TalonSRX(Const.climbMotorPort);
        climbServo = new Servo(Const.climbServoPort);
        slideMotor = new TalonSRX(Const.climbSlideMotor);
        //魔法の煙を出さないように
        slideMotor.configContinuousCurrentLimit(12);
        slideMotor.enableCurrentLimit(true);

        //コントローラーの初期化
        operator = new XboxController(Const.OperateControllerPort);
        driver = new XboxController(Const.DriveControllerPort);

        //cameraの初期化
        driveCamera = CameraServer.getInstance();
        armCamera = CameraServer.getInstance();
        driveCamera.startAutomaticCapture("drive", 1);
        armCamera.startAutomaticCapture("arm", 0);

        //ドライブモーターの初期化
        driveRightFrontMotor = new WPI_TalonSRX(Const.DriveRightFrontPort);
        driveRightBackMotor = new VictorSPX(Const.DriveRightBackPort);
        driveLeftFrontMotor = new WPI_TalonSRX(Const.DriveLeftFrontPort);
        driveLeftBackMotor = new VictorSPX(Const.DriveLeftBackPort);

        //ドライブモーターの台形加速&フォローの設定
        driveLeftFrontMotor.configOpenloopRamp(Const.DriveFullSpeedTime);
        driveLeftBackMotor.follow(driveLeftFrontMotor);
        driveRightFrontMotor.configOpenloopRamp(Const.DriveFullSpeedTime);
        driveRightBackMotor.follow(driveRightFrontMotor);

        //シューターの設定を初期化
        shooterRightMotor.configFactoryDefault();
        shooterLeftMotor.configFactoryDefault();

        //シューターのPIDの設定
        shooterLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                Const.kPIDLoopIdx,
                Const.kTimeoutMs);
        shooterRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                Const.kPIDLoopIdx,
                Const.kTimeoutMs);

        shooterLeftMotor.config_kF(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kF, Const.kTimeoutMs);
        shooterLeftMotor.config_kP(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kP, Const.kTimeoutMs);
        shooterLeftMotor.config_kI(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kI, Const.kTimeoutMs);
        shooterLeftMotor.config_kD(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kD, Const.kTimeoutMs);

        shooterRightMotor.config_kF(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kF, Const.kTimeoutMs);
        shooterRightMotor.config_kP(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kP, Const.kTimeoutMs);
        shooterRightMotor.config_kI(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kI, Const.kTimeoutMs);
        shooterRightMotor.config_kD(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.kD, Const.kTimeoutMs);

        shooterLeftMotor.configMaxIntegralAccumulator(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.MaxIntegralAccumulator);
        shooterRightMotor.configMaxIntegralAccumulator(Const.kPIDLoopIdx, Const.kGains_ShooterVelocity.MaxIntegralAccumulator);
        shooterRightMotor.setSensorPhase(true);
        shooterLeftMotor.setSensorPhase(true);

        //Armの設定を初期化
        armMotor.configFactoryDefault();

        //ArmのPID設定
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog,
                Const.kArmPIDLoopIdx,
                Const.kTimeoutMs);

        armMotor.config_kF(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kF, Const.kTimeoutMs);
        armMotor.config_kP(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kP, Const.kTimeoutMs);
        armMotor.config_kI(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kI, Const.kTimeoutMs);
        armMotor.config_kD(Const.kArmPIDLoopIdx, Const.kGains_ArmPosition.kD, Const.kTimeoutMs);

        armMotor.configMaxIntegralAccumulator(Const.kPIDLoopIdx, Const.kGains_ArmPosition.MaxIntegralAccumulator);

        armMotor.setSensorPhase(true);
        armMotor.setInverted(true);
        /*
        初期値が確認できたら、削除予定
        ShooterLeft.configNominalOutputForward(0, Const.kTimeoutMs);
        shooterLeft.configNominalOutputReverse(0, Const.kTimeoutMs);
        shooterLeft.configPeakOutputForward(1, Const.kTimeoutMs);
        shooterLeft.configPeakOutputReverse(-1, Const.kTimeoutMs);
        */

        /*
        初期値が確認できたら、削除予定
        shooterRight.configNominalOutputForward(0, Const.kTimeoutMs);
        shooterRight.configNominalOutputReverse(0, Const.kTimeoutMs);
        shooterRight.configPeakOutputForward(1, Const.kTimeoutMs);
        shooterRight.configPeakOutputReverse(-1, Const.kTimeoutMs);
         */

        //サブクラスの生成
        armSensor = new ArmSensor(armMotor);
        arm = new Arm(armMotor, armEncoder, armSensor);
        drive = new Drive(driveLeftFrontMotor, driveRightFrontMotor);
        shooter = new Shooter(shooterRightMotor, shooterLeftMotor);
        intake = new Intake(intakeMotor);
        intakeBelt = new IntakeBelt(intakeBeltFrontMotor, intakeBeltBackMotor, intakeFrontSensor, intakeBackSensor);
        state = new State();


        //モードのクラスの生成
        panelRotationMode = new PanelRotationMode(colorSensorServo);
        climbMode = new ClimbMode(arm, climbMotor, climbServo, slideMotor);
    }

    @Override
    public void robotPeriodic() {
        //処理なし
    }

    @Override
    public void autonomousInit() {
        autonomousTimer = new Timer();
        autonomousTimer.reset();
        autonomousTimer.start();
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        panelRotationMode.contractServo();
    }

    @Override
    public void autonomousPeriodic() {

        state.stateInit();

        drive.applyState(state);
        arm.applyState(state);
        shooter.applyState(state);
        intake.applyState(state);
        intakeBelt.applyState(state);

    }

    public void teleopInit() {
        state.controlMode = State.ControlMode.m_Drive;
        panelRotationMode.contractServo();
    }

    @Override
    public void teleopPeriodic() {

        System.out.println("leftSpeed" + shooterLeftMotor.getSelectedSensorVelocity());
        System.out.println("RightSpeed" + shooterRightMotor.getSelectedSensorVelocity());

        //状態初期化
        state.stateInit();
        state.armAngle = arm.getArmNow();

        state.shooterLeftMotorSpeed = shooterLeftMotor.getSelectedSensorVelocity();
        state.shooterRightMotorSpeed = shooterRightMotor.getSelectedSensorVelocity();

        //Mode Change
        switch (state.controlMode) {
            case m_Drive:
                if (operator.getBumper(GenericHID.Hand.kLeft)) {
                    //O LB ボール発射モードへ切り替え
                    state.controlMode = State.ControlMode.m_ShootingBall;
                } else if (driver.getBackButton()) {
                    //D Back コントロールパネル回転モードへ切り替え
                    state.controlMode = State.ControlMode.m_PanelRotation;
                    state.armTargetAngle = state.armAngle;
                    //state.armTargetAngle = State.armAngle;
                    //state.targetAngle = State.armAngle;
                } else if (operator.getBackButton()) {
                    //O Backクライムモードへ切り替え
                    state.controlMode = State.ControlMode.m_Climb;
                }
                break;

            case m_Climb:
            case m_ShootingBall:
                if (driver.getStartButton() || operator.getStartButton()) {
                    //D or O Start ドライブモードへ
                    state.controlMode = State.ControlMode.m_Drive;
                    state.climbWireState = State.ClimbWireState.climbLock;
                }
                break;
            case m_PanelRotation:
                if (driver.getStartButton() || operator.getStartButton()) {
                    //D or O Start ドライブモードへ
                    panelRotationMode.contractServo();
                    if (!servoTimerStarted) {
                        servoTimer.reset();
                        servoTimer.start();
                        servoTimerStarted = true;
                    }
                    System.out.println(colorSensorServo.getAngle());
                    if (servoTimer.get() > 1) {
                        state.controlMode = State.ControlMode.m_Drive;
                        servoTimerStarted = false;
                    }
                }
                break;
        }
        System.out.println(colorSensorServo.getAngle());
        Util.sendConsole("Mode", state.controlMode.toString());

        switch (state.controlMode) {
            case m_Drive:
                //ほかに関係なくドライブ
                if (driver.getBumper(GenericHID.Hand.kLeft)) {
                    //D LBで低速モード
                    state.driveState = State.DriveState.kLow;
                } else {
                    state.driveState = State.DriveState.kManual;
                }

                state.driveStraightSpeed = Util.deadbandProcessing(-driver.getY(GenericHID.Hand.kLeft));
                state.driveRotateSpeed = Util.deadbandProcessing(driver.getX(GenericHID.Hand.kRight));

                state.intakeState = State.IntakeState.kDrive;

                if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kLeft))) {
                    //D LT ボールを取り込む
                    state.intakeState = State.IntakeState.kIntake;
                    state.intakeBeltState = State.IntakeBeltState.kIntake;
                    state.shooterState = State.ShooterState.kIntake;
                } else if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //D RT ボールを出す
                    state.intakeState = State.IntakeState.kOuttake;
                    state.intakeBeltState = State.IntakeBeltState.kOuttake;
                    state.shooterState = State.ShooterState.kOuttake;
                } else if (driver.getBumper(GenericHID.Hand.kRight)) {
                    //D RB アームを平行（パネルくぐり）
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Const.armParallelAngle;
                    state.intakeState = State.IntakeState.doNothing;
                }
                break;

            case m_ShootingBall:
                //Armは調整しない限り動かない
                state.armState = State.ArmState.k_Conserve;
                //D Stick ドライブを少し動かす
                state.driveState = State.DriveState.kLow;
                state.driveStraightSpeed = -driver.getY(GenericHID.Hand.kLeft);
                state.driveRotateSpeed = driver.getX(GenericHID.Hand.kRight);
                if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //O RT ボールを飛ばす
                    state.shooterState = State.ShooterState.kShoot;
                    state.intakeBeltState = State.IntakeBeltState.kOuttake;
                } else if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))) {
                    //O LT 60度に角度調整//
                    state.armState = State.ArmState.k_ConstAng;
                    state.armFinalTargetAngle = 35;
                } else if (operator.getYButton()) {
                    //O Y　砲台の角度調節（InitialLineにあるとき）
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Const.armShootInitiationAngle;
                } else if (operator.getAButton()) {
                    //O A　砲台の角度調節（真下にある時、下へ）
                    state.armState = State.ArmState.k_PID;
                    state.armSetAngle = Const.armParallelAngle;
                } else if (Util.deadbandCheck(operator.getY(GenericHID.Hand.kLeft))) {
                    //O LStick Y 砲台の角度を手動で調節, 正か負のみ
                    state.armState = State.ArmState.k_Adjust;
                    state.armMotorSpeed = -operator.getY(GenericHID.Hand.kLeft);
<<<<<<< HEAD
                } 
                    /*if(Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))&&Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))){
                        state.intakeBeltState = State.IntakeBeltState.kouttake;
                    }*/
=======
                } else if (operator.getBButton()) {
                    //O B 60度に角度調整//
                    state.armState = State.ArmState.k_ConstAng;
                    state.armSetAngle = 60; //後で変更予定
                    state.armFinalTargetAngle = 60;
                }

                /*
                * if(Util.deadandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))&&Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))){
                * state.intakeBeltState = State.IntakeBeltState.kouttake;
                * }
                */

>>>>>>> e87832254dacea06429288d8f63e23f9309b24e8
                break;

            case m_Climb:
                //Drive
                state.driveState = State.DriveState.kLow;
                state.driveStraightSpeed = Util.deadbandProcessing(-driver.getY(GenericHID.Hand.kLeft));
                state.driveRotateSpeed = Util.deadbandProcessing(driver.getX(GenericHID.Hand.kRight));

                //Climb
                state.armState = State.ArmState.k_Conserve;
                if (operator.getYButton()) {
                    //O Y アームを上げる
                    state.climbArmState = State.ClimbArmState.climbExtend;
                    state.climbExtendAdjustSpeed = -operator.getY(GenericHID.Hand.kLeft) / 3.5;
                } else if (operator.getBButton()) {
                    //O B クライムする(本番)
                    state.climbWireState = State.ClimbWireState.climbShrink;
                } else if (Util.deadbandCheck(operator.getX(GenericHID.Hand.kRight))) {
                    //O RStick X スライド
                    state.climbArmState = State.ClimbArmState.climbSlide;
                    if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kLeft))) {
                        //O LT 高出力でスライド
                        state.climbSlideMotorSpeed = -operator.getX(GenericHID.Hand.kLeft);
                    } else {
                        //1/2の出力でスライド
                        state.climbSlideMotorSpeed = operator.getX(GenericHID.Hand.kRight) / 2;
                    }
                }
                if (operator.getBumper(GenericHID.Hand.kRight)) {
                    //O RB Climb Motorだけ縮める(調整用)
                    state.climbWireState = State.ClimbWireState.climbMotorOnlyShrink;
                } else if (operator.getBumper(GenericHID.Hand.kLeft)) {
                    //O LB Climb　Motorだけ伸ばす(調整用)
                    state.climbWireState = State.ClimbWireState.climbMotorOnlyExtend;
                } else if (Util.deadbandCheck(operator.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //O RT ロックする
                    state.climbWireState = State.ClimbWireState.climbLock;
                }

                climbMode.changeState(state);
                break;

            case m_PanelRotation:
                //Drive
                state.driveStraightSpeed = Util.deadbandProcessing(-driver.getY(GenericHID.Hand.kLeft));
                state.driveRotateSpeed = Util.deadbandProcessing(driver.getX(GenericHID.Hand.kRight));

                //Arm
                state.armState = State.ArmState.k_PID;
                state.armSetAngle = Const.armPanelAngle;

                if (driver.getXButton()) {
                    //D X 青に合わせる
                    state.panelState = State.PanelState.p_toBlue;
                } else if (driver.getYButton()) {
                    //D Y 黄に合わせる
                    state.panelState = State.PanelState.p_toYellow;
                } else if (driver.getBButton()) {
                    //D B 赤に合わせる
                    state.panelState = State.PanelState.p_toRed;
                } else if (driver.getAButton()) {
                    //D A 緑に合わせる
                    state.panelState = State.PanelState.p_toGreen;
                } else if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kLeft))) {
                    //D LT 手動左回転
                    state.panelState = State.PanelState.p_ManualRot;
                    state.panelManualSpeed = -Const.shooterPanelManualSpeed;
                } else if (Util.deadbandCheck(driver.getTriggerAxis(GenericHID.Hand.kRight))) {
                    //D RT 手動右回転
                    state.panelState = State.PanelState.p_ManualRot;
                    state.panelManualSpeed = Const.shooterPanelManualSpeed;
                }
                panelRotationMode.changeState(state);
                break;
        }

        drive.applyState(state);
        arm.applyState(state);
        shooter.applyState(state);
        intake.applyState(state);
        intakeBelt.applyState(state);

    }

    @Override
    public void testPeriodic() {
    }
}