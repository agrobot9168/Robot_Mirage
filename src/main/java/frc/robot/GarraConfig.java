 package frc.robot;

 import com.ctre.phoenix6.hardware.TalonFX;
 import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.MathUtil;

   public class GarraConfig {
    // TalonFXSConfiguration configs = new TalonFXSConfiguration();

    // configs.Commutation.MotorArrangement
    // MotorArrangementValue.Minion_JST; 

       public TalonFXS motorGarra;

      //e static final double SAIDA_MAXIMA = 0.15;

      public GarraConfig() {
           motorGarra = new TalonFXS(14);
       
       TalonFXSConfiguration configs = new TalonFXSConfiguration();
       configs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

       motorGarra.getConfigurator().apply(configs); 
      }


      public void setMotor(double velocidade) {
           motorGarra.set(velocidade);
       }

       public void stopMotor() {
        motorGarra.set(0);
       }

       public double getCurrent() {
        return motorGarra.getStatorCurrent().getValueAsDouble();
       }

       public double correnteAlvo = 10.;

       public double calcularCorrecaoCorrente() {
      
         double correnteAtual = getCurrent();

         return   pid_FXS.calculate(correnteAtual, correnteAlvo);
       }

       public double calcularSaidaLimitada() {

         double correcao = calcularCorrecaoCorrente();
         return correcao;
       //return MathUtil.clamp(correcao, 0, 30);
       }
    //==========================
    // CONTROLES
    // =========================

    public final PIDController pid_FXS = new PIDController(0.1, 0, 0);

   } 

