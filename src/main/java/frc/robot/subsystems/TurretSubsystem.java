package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase{
    private String limelightaugust = "limelight-august";
    private double alvoRPS = 0.0;
    private double angle;
    private double shutter;

    private final TalonFX mShutter;
    private final VelocityVoltage pedidoVelocidade;

    private final SparkMax mInclina_shutter;
    private final SparkMaxConfig cfg_InclinaShutter;
    private SparkClosedLoopController pid_InclinaShutter;

    public TurretSubsystem() {
        mShutter = new TalonFX(20);
        pedidoVelocidade = new VelocityVoltage(0).withSlot(0);

        mInclina_shutter = new SparkMax(22, MotorType.kBrushless);
        cfg_InclinaShutter = new SparkMaxConfig();
        pid_InclinaShutter = mInclina_shutter.getClosedLoopController();

        SetReference_InclinaShutter();
        CfgPid_Inclina(0.1);
    }

    public double GetPose_InclinaShutter() { // function for get encoder position 
        return mInclina_shutter.getEncoder().getPosition();
    }

    public void CfgPid_Inclina(double kp) { // function for config PID
        cfg_InclinaShutter.smartCurrentLimit(20);
        cfg_InclinaShutter.inverted(false).idleMode(IdleMode.kBrake);
        cfg_InclinaShutter.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfg_InclinaShutter.closedLoop
        .pid(kp, 0, 0)
        .outputRange(-0.25, 0.25);

        mInclina_shutter.configure(cfg_InclinaShutter, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        pid_InclinaShutter = mInclina_shutter.getClosedLoopController();
    }

    public void SetReference_InclinaShutter() { //function for set reference = 0 
        mInclina_shutter.getEncoder().setPosition(0);
    }
     public void SetPose_Inclina(double setPoint) { //function for set position
        pid_InclinaShutter.setSetpoint(setPoint, ControlType.kPosition);
    }
    public void SetRPM_Shutter(double setRPM) {
        this.alvoRPS = setRPM / 60.0;
        double meuKv = 0.115;
        double VoltagemFeedForward = this.alvoRPS * meuKv;
        mShutter.setControl(pedidoVelocidade.withVelocity(alvoRPS).withFeedForward(VoltagemFeedForward));
    }
    public double GetRPM_Shutter() {
        return mShutter.getVelocity().getValueAsDouble();
    }
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    public double Angle() {
        return map(GetDistance(), 1, 3, -2, -7);
    }
    public double VelShutter() {
        return map(GetDistance(), 1, 3, 1750, 2050);
    }
    public double GetDistance() {
       

        Pose3d target = LimelightHelpers.getTargetPose3d_RobotSpace(limelightaugust);
        double z = target.getZ();

        return z;
    }
    
    @Override
    public void periodic() {
        Pose3d target = LimelightHelpers.getTargetPose3d_RobotSpace(limelightaugust);

        double x = target.getX();
        double z = target.getZ();
        double y = target.getY();

        SmartDashboard.putNumber("GetX", x);
        SmartDashboard.putNumber("GetY", y);
        SmartDashboard.putNumber("GetZ", z);
        SmartDashboard.putNumber("GetInclina_Shutter", GetPose_InclinaShutter());
        SmartDashboard.putNumber("Seno", GetDistance());

        SmartDashboard.putNumber("GetTX", LimelightHelpers.getTX(limelightaugust));
        SmartDashboard.putNumber("GetTY", LimelightHelpers.getTY(limelightaugust));

        SmartDashboard.putNumber("VelShutter", GetRPM_Shutter() * 60);


    }

}