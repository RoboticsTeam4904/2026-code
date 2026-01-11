package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

/** Orchestra™ */
public class OrchestraSubsystem extends SubsystemBase {

    private static final HashMap<String, OrchestraSubsystem> songs = new HashMap<>();

    /**
     * Loads a song to be played later with {@link OrchestraSubsystem#playSong(String)}.
     * <p>
     * Files for the song should be located at {@code deploy/chirp/<name>_<track>.chrp}
     *
     * @param name The name of the song to load.
     * @param tracks The number of tracks the song has.
     * @param motors The motors that will be used to play the song. There should be at least as many motors as there are tracks.
     *               Motors will be distributed as evenly as possible between the tracks, filling all tracks first, then going
     *               back to add multiple motors to the same track (you can use nulls to skip over tracks when assigning motors).
     */
    public static void loadSong(String name, int tracks, CANTalonFX... motors) {
        if (songs.get(name) != null) {
            System.out.println("Song '" + name + "' already exists");
        } else {
            songs.put(name, new OrchestraSubsystem(name, tracks, motors));
        }
    }

    /**
     * Play a song that was already loaded with {@link OrchestraSubsystem#loadSong(String, int, CANTalonFX...)}.
     * @param name The name of the song to play.
     * @return Whether the song was successfully played.
     */
    public static boolean playSong(String name) {
        stopAll();

        var orchestra = songs.get(name);

        if (orchestra == null) {
            System.out.println("Song '" + name + "' does not exist");
            return false;
        }

        return orchestra.play();
    }

    /**
     * Stop a song.
     * @param name The name of the song to stop.
     * @return Whether the song was successfully stopped.
     */
    public static boolean stopSong(String name) {
        var orchestra = songs.get(name);

        if (orchestra == null) {
            System.out.println("Song '" + name + "' does not exist");
            return false;
        }

        return orchestra.stop();
    }

    /**
     * Stop all songs.
     * @return Whether all songs were successfully stopped.
     */
    public static boolean stopAll() {
        boolean success = true;

        for (var song : songs.values()) {
            if (song.playing && !song.stop()) {
                success = false;
            }
        }

        return success;
    }

    /**
     * Loads a song and then returns a command to play it.
     * See {@link OrchestraSubsystem#loadSong(String, int, CANTalonFX...)}.
     * @return A {@link Command} to play the song.
     */
    public static Command c_loadAndPlaySong(String name, int tracks, CANTalonFX... motors) {
        loadSong(name, tracks, motors);
        return new InstantCommand(() -> playSong(name));
    }

    public static final String PATH = "/home/lvuser/deploy/chirp/";

    public final String songName;
    public boolean playing = false;

    private final List<Orchestra> orchestras = new ArrayList<>();

    private OrchestraSubsystem(String name, int tracks, CANTalonFX... motors) {
        songName = name;

        if (motors.length < tracks) {
            System.out.printf(
                "Not enough motors for song '%s' (Got: %d, Recommended: %d).%n",
                name,
                motors.length,
                tracks
            );
        }

        // create a separate instance of Orchestra for each track because we can't
        // figure out how to put multiple tracks into one .chrp file :(
        for (int track = 0; track < tracks; track++) {
            var orchestra = new Orchestra();

            String path = PATH + name + "_" + track + ".chrp";
            if (!orchestra.loadMusic(path).isOK()) {
                System.err.printf(
                    "Failed to load song '%s', track %d. Make sure the file '%s' exists.%n",
                    path,
                    track,
                    path
                );
            }

            for (int i = track; i < motors.length; i += tracks) {
                CANTalonFX motor = motors[i];
                if (motor != null) orchestra.addInstrument(motor.motor, 0);
            }

            orchestras.add(orchestra);
        }
    }

    private boolean play() {
        playing = true;

        boolean success = true;

        for (int i = 0; i < orchestras.size(); i++) {
            if (!orchestras.get(i).play().isOK()) {
                System.out.println("Song '" + songName + "', track " + i + " failed to play.");
                success = false;
            }
        }

        return success;
    }

    private boolean stop() {
        if (!this.playing) return false;

        playing = false;

        boolean success = true;

        for (int i = 0; i < orchestras.size(); i++) {
            if (!orchestras.get(i).stop().isOK()) {
                System.out.println("Song '" + songName + "', track " + i + " failed to stop.");
                success = false;
            }
        }

        return success;
    }
}
