// package robot.commands.net;

// import java.util.concurrent.TimeUnit;

// import robot.subsystems.Turret;
// import robot.subsystems.net.RobotUDP;
// import robot.subsystems.net.messages.OdometryUpdate;
// import lib.util.Util;
// import lib.custom.sensors.NavX;
// import lib.subsystems.chassis.SensorDrive;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj2.command.Command;

// public class OdometrySend extends Command {
//     private final static double SECS_PER_MICROSEC = Util.timeConversionFactor(TimeUnit.MICROSECONDS, TimeUnit.SECONDS);

//     private final RobotUDP net;

//     private final SensorDrive sensorDrive;
//     private final NavX navx;
//     private final Turret turret;

//     private long lastTimestamp;

//     public OdometrySend(RobotUDP net, SensorDrive sensorDrive, NavX navx, Turret turret) {
//         this.net = net;

//         this.sensorDrive = sensorDrive;
//         this.navx = navx;
//         this.turret = turret;
//     }

//     @Override
//     public void execute() {
//         final long timestamp = RobotController.getFPGATime();

//         final double dt = lastTimestamp == 0 ? 0 : (timestamp - lastTimestamp) * SECS_PER_MICROSEC;
//         final var accelYawDegrees = dt == 0 ? 0 : navx.getRate() / dt;

//         final var accel = new Pose2d(
//                 navx.getWorldLinearAccelX(),
//                 navx.getWorldLinearAccelY(),
//                 Rotation2d.fromDegrees(accelYawDegrees));

//         final var update = new OdometryUpdate(
//                 sensorDrive.getPose(),
//                 accel,
//                 turret.getAngle(),
//                 timestamp);

//         net.updateOdometry(update);
//         lastTimestamp = timestamp;
//     }
// }
