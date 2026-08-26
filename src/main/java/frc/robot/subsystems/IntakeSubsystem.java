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

    private final SparkMax mInclina;
    private final SparkMax mColeta;

    private final SparkMaxConfig cfgInclina;
    private SparkClosedLoopController pidInclina;


    public IntakeSubsystem() {
        mInclina = new SparkMax(14, MotorType.kBrushless);
        mColeta = new SparkMax(15, MotorType.kBrushless);
        cfgInclina = new SparkMaxConfig();
        pidInclina = mInclina.getClosedLoopController();

        SetReference();
        CfgPid_Inclina(0.3);
    }

    public double GetPose_Inclina() {
        return mInclina.getEncoder().getPosition();
    }
    public void SetReference() {
        mInclina.getEncoder().setPosition(0);
    }
    public void SetVel_Colet(double vel) {
        mColeta.set(vel);
    }
    public void CfgPid_Inclina(double kp) {
        cfgInclina.smartCurrentLimit(20);
        cfgInclina.inverted(false).idleMode(IdleMode.kBrake);
        cfgInclina.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        cfgInclina.closedLoop
        .pid(kp, 0.0, 0.0)
        .outputRange(-0.4, 0.4);

        mInclina.configure(cfgInclina, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        pidInclina = mInclina.getClosedLoopController();
  }

  public void SetPose_Inclina(double setPoint) {
    pidInclina.setSetpoint(setPoint, ControlType.kPosition);
  }
  @Override
  public void periodic() {
      SmartDashboard.putNumber("GetPose_Intake", GetPose_Inclina());
  }
}
