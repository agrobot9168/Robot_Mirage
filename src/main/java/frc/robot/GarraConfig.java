 package frc.robot;

   import com.ctre.phoenix6.hardware.TalonFXS;

   public class GarraConfig {

       public TalonFXS motorGarra;

      public GarraConfig() {
           motorGarra = new TalonFXS(14);
       } 

      public double getCurrent() {
          return motorGarra.getStatorCurrent().getValueAsDouble();
       }
   } 

// package frc.robot;

// public class GarraConfig {
//      public GarraConfig() {
//         final double motorGarra = 0.0;// new TalonFXS(14);
//      }
// }