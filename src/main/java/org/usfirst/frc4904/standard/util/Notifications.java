package org.usfirst.frc4904.standard.util;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.Elastic;
import org.usfirst.frc4904.standard.commands.AlwaysRunnableInstantCommand;
import org.usfirst.frc4904.standard.Elastic.Notification;
import org.usfirst.frc4904.standard.Elastic.NotificationLevel;

import java.util.Random;

public final class Notifications {

    private Notifications() {}

    public static Command c_testNotif() {
        var notif = new Notification();
        notif.setTitle("Cheese");
        notif.setDescription("try some.");
        return c_sendNotification(notif);
    }

    private static final String[] randomNotifOptions = {
        "You look very daunted",
        "Are you trying to bring back 4904 ref provoking?",
        "WARNING: driving subpar",
        "You'll always be Alon's son",
        "Are you no longer noteworthy???",
        "He's behind you",
        "WARNING: TalonFX on CAN ID 6 not responding",
        "WARNING: Self destruct in 10 seconds",
        "I am in your walls",
        "Minor malfunction found",
        "You're doing good, son",
        "Nice work!",
        "Have you had some cheese today?",
        "Ben. Ben BEn BEN.",
        "Robot has been possessed by 5940",
        "Wow, such a big strong",
        "WARNING: robor is cooked"
    };

    public static Command c_sendRandom() {
        return c_randomNotif(randomNotifOptions, NotificationLevel.WARNING);
    }

    private static final Random random = new Random();

    /**
     * Send a random Elastic™ notification to the dashboard
     *
     * @param options An array of possible notifications
     * @param level a {@link NotificationLevel} that the notification displays as
     */
    public static void randomNotif(String[] options, NotificationLevel level) {
        String title = options[random.nextInt(options.length)];
        Elastic.sendNotification(new Notification(level, title, ""));
    }

    // command versions of notification methods

    /** Command-based version of {@link #randomNotif(String[], NotificationLevel)} */
    public static Command c_randomNotif(String[] options, NotificationLevel level) {
        return new AlwaysRunnableInstantCommand(() -> randomNotif(options, level));
    }

    /** Command-based version of {@link Elastic#sendNotification(Notification)} */
    public static Command c_sendNotification(Notification notification) {
        return new AlwaysRunnableInstantCommand(() -> Elastic.sendNotification(notification));
    }

}
