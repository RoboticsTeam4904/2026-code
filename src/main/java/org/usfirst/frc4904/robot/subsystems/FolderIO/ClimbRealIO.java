package org.usfirst.frc4904.robot.subsystems.FolderIO;
    
import edu.wpi.first.wpilibj.motorcontrol.Spark;
    
    public class ClimbRealIO implements ClimbIO{
    
        Spark spark = new Spark(1);
        //Encoder encoder
    
        @Override
        public ClimbState.InputState getInstance() {
            //get encoder value return here
            return null;
        }
    
        @Override
        public void setState(ClimbState.OutputState output) {
            //spark.setVolatge(output.voltage());
        }
    }

    //Defining the real inputs and outputs


