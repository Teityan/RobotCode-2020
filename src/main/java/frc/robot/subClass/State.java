package frc.robot.subClass;

public class State {

    //速度
    //Drive
    public double driveStraightSpeed, driveRotateSpeed;
    //Shooter
    public double shooterLeftSpeed, shooterRightSpeed;
    public double shooterPIDSpeed;
    public double panelManualSpeed;
    public double shooterLeftMotorSpeed , shooterRightMotorSpeed;
    //Arm
    public double armMotorSpeed;
    //Climb
    public double climbSlideMotorSpeed;
    public double climbExtendAdjustSpeed;

    //Arm Angle
    public double armAngle;
    public double armSetAngle;
    public double DisAng;
    public double armFinalTargetAngle;
    public double targetAngle;
    public double armTargetAngle;
    
    //SubClass State
    public DriveState driveState;
    public ArmState armState;
    public ShooterState shooterState;
    public IntakeState intakeState;
    public IntakeBeltState intakeBeltState;
    public ClimbArmState climbArmState;
    public ClimbWireState climbWireState;
    public PanelState panelState;

    //Control Mode
    public ControlMode controlMode = ControlMode.m_Drive;

    //ボールを5個ゲットしたか
    public boolean is_IntakeFull;

    public State() {
        stateInit();
    }

    public void stateInit() {

        //Drive
        driveState = DriveState.kManual;
        driveStraightSpeed = 0;
        driveRotateSpeed = 0;

        //Shooter
        shooterState = ShooterState.doNothing;
        shooterLeftSpeed = 0;
        shooterRightSpeed = 0;
        shooterPIDSpeed = 0;
        shooterRightMotorSpeed = 0;
        shooterLeftMotorSpeed = 0;

        //Intake
        intakeState = IntakeState.doNothing;
        is_IntakeFull = false;

        //IntakeBeltState
        intakeBeltState = IntakeBeltState.doNothing;

        //Climb
        climbArmState = ClimbArmState.doNothing;
        climbWireState = ClimbWireState.doNothing;
        climbSlideMotorSpeed = 0;
        climbExtendAdjustSpeed = 0;

        //Arm
        armState = ArmState.k_Basic;
        armMotorSpeed = 0;
        armSetAngle = Const.armMinAngle;
        armAngle = 0;
        armTargetAngle = 0;

        //panel
        panelState = PanelState.p_DoNothing;
        panelManualSpeed = 0;
    }

    public enum ControlMode {
        m_ShootingBall,
        m_PanelRotation,
        m_Climb,
        m_Drive
    }

    public enum DriveState {
        kManual,
        kLow,
        kSuperLow,
        kStop,
        kMiddleLow
    }

    public enum ShooterState {
        kShoot,
        kIntake,
        kManual,
        doNothing,
        kOuttake
    }

    public enum IntakeState {
        kIntake,
        kOuttake,
        doNothing,
        kDrive
    }

    public enum IntakeBeltState {
        kIntake,
        kOuttake,
        doNothing
    }

    public enum ClimbArmState {
        doNothing,
        climbExtend,
        climbSlide
    }

    public enum ClimbWireState {
        doNothing,
        climbMotorOnlyExtend,
        climbMotorOnlyShrink,
        climbShrink,
        climbLock
    }

    public enum ArmState {
        k_Conserve,
        k_Adjust,
        k_PID,
        k_Basic,
        k_Manual,
        k_ConstAng,
        k_DoNothing,
    }

    public enum PanelState {
        p_DoNothing,        //stop
        p_ManualRot,        //手動
        p_toBlue,           //色合わせ(大会のパネル側のセンサーの色なのでカラーセンサーが読み取るのは二つずれた値。青<->赤、黄<->緑）
        p_toYellow,
        p_toRed,
        p_toGreen

    }

}