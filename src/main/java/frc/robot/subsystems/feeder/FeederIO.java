// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public  interface FeederIO {
    @AutoLog
     public static class FeederIOInputs {

     }

        public default void setFeederVolts(){}

}
