package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase{
    private double kP = 0.0005;
    private double kV = 0.00020;

    private final SparkMax mInclina;
    private final SparkMax mColeta;

    private final SparkMaxConfig cfgInclina;
    private final SparkMaxConfig cfgColet;

    private SparkClosedLoopController pidInclina;
    private SparkClosedLoopController pidColet;


    public IntakeSubsystem() {
        mInclina = new SparkMax(14, MotorType.kBrushless);
        mColeta = new SparkMax(15, MotorType.kBrushless);

        cfgColet = new SparkMaxConfig();
        cfgInclina = new SparkMaxConfig();
        
        pidInclina = mInclina.getClosedLoopController();
        
        SetReference();
        setReference_Colet();
        CfgPid_Inclina(0.8);
        // CfgPid_Colet(2);
        Cfg_Rpm_Colet();
    }

    public double GetPose_Inclina() {
        return mInclina.getEncoder().getPosition();
    }
    // public double GetPose_Colet() {
    //     return mColeta.getEncoder().getPosition();
    // }
    public void SetReference() {
        mInclina.getEncoder().setPosition(0);
    }
    public void setReference_Colet() {
      mColeta.getEncoder().setPosition(0);
    }
    public void SetVel_Colet(double vel) {
        mColeta.set(vel);
    }
    public void Cfg_Rpm_Colet() {
      cfgColet.smartCurrentLimit(30).advanceCommutation(-120);
      cfgColet.inverted(false).idleMode(IdleMode.kCoast);
      cfgColet.encoder.velocityConversionFactor(1);
      cfgColet.closedLoop.pid(0.000, 0, 0);
      mColeta.configure(cfgColet, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
      pidColet = mColeta.getClosedLoopController();
    }
    public void CfgPid_Inclina(double kp) {
        cfgInclina.smartCurrentLimit(20);
        cfgInclina.inverted(false).idleMode(IdleMode.kBrake);
        cfgInclina.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgInclina.closedLoop
        .pid(kp, 0.0, 0.0)
        .outputRange(-0.3, 0.7);

        mInclina.configure(cfgInclina, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        pidInclina = mInclina.getClosedLoopController();
  }
  public void SetRPM_Colet(double RPM) {
    pidColet.setSetpoint(RPM, ControlType.kVelocity);
  }
  public double GetRPM_Colet() {
    return mColeta.getEncoder().getVelocity();
  }
  public void CfgPid_Colet(double kp) {
        cfgColet.smartCurrentLimit(30);
                                                    // .advanceCommutation(-120);
        cfgColet.inverted(false).idleMode(IdleMode.kBrake);
        cfgColet.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgColet.closedLoop
        .pid(kp, 0.0, 0.0)
        .outputRange(-1, 1);

        mColeta.configure(cfgColet, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        pidColet = mColeta.getClosedLoopController();
  }

  public void SetPose_Inclina(double setPoint) {
    pidInclina.setSetpoint(setPoint, ControlType.kPosition);
  }
  // public void SetPose_Colet(double setPoint) {
  //   pidColet.setSetpoint(setPoint, ControlType.kPosition);
  // }
  // public void SetRPM_Guloso(double RPM_Alvo) {
  //   double rpmAtual = GetRPM_Colet();

  //   double erro = RPM_Alvo - rpmAtual;

  //   double feedForward = RPM_Alvo * kV;

  //   double correcao = erro * kP;

  //   double potencia = feedForward + correcao;

  //   potencia = Math.max(-1.0, Math.min(1.0, potencia));

  //   mColeta.set(potencia);
  // }
  @Override
  public void periodic() {
      SmartDashboard.putNumber("GetPose_Intake", GetPose_Inclina());
      // SmartDashboard.putNumber("GetPose_Coleta", GetPose_Colet());
      SmartDashboard.putNumber("getName()", GetRPM_Colet());
  }
}
