package org.usfirst.frc4904.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonProcessingException;
import edu.wpi.first.math.geometry.*;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.usfirst.frc4904.standard.util.Util;

import javax.swing.text.html.Option;

/** Manages Google tags */
public final class GoogleTagManager {

    private GoogleTagManager() {}

    public record Tag(int id, Rotation2d rot, Translation3d pos, Pose2d fieldPos, double time, int camera) {}

    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static final ObjectMapper mapper = new ObjectMapper();
    private static final NetworkTableEntry tagsEntry, timeEntry;
    static {
        var table = NetworkTableInstance.getDefault();
        tagsEntry = table.getEntry("/dauntless/tags");
        timeEntry = table.getEntry("/dauntless/time");
    }

    private static double lastTime;
    private static List<Tag> lastTags;

    public static List<Tag> getTags() {
        double timeSeconds = timeEntry.getDouble(0);

        // TODO reinstate once time networktable entry is implemented
        // if (lastTime == timeSeconds && lastTags != null) return lastTags;
        lastTime = timeSeconds;

        double time = Util.epochSecondsToFPGATimestamp(timeSeconds);

        List<Tag> tags = lastTags = new ArrayList<>();

        String json = tagsEntry.getString("");
        if (json.isEmpty()) return tags;

        try {
            JsonNode root = mapper.readTree(json);

            for (JsonNode el : root) {
                JsonNode idPath = el.path("id");
                if (idPath.isNull()) continue;
                int id = idPath.asInt();

                double[] pos = mapper.treeToValue(el.path("pos"), double[].class);

                Optional<Pose3d> tagPose = field.getTagPose(1 /* TODO temporary */);
                if (tagPose.isEmpty()) {
                    System.err.println("Tag id " + id + " does not exist on field layout");
                    continue;
                }

                tags.add(new Tag(
                    id,
                    Rotation2d.fromRotations(el.path("rot").asDouble()),
                    new Translation3d(pos[2], pos[0], pos[1]),
                    tagPose.get().toPose2d(),
                    time,
                    0
                ));
            }
        } catch (JsonProcessingException e) {
            System.out.println("google tag manager parsing error!!!");
        }

        return tags;
    }

    /**
     * For usage with {@link #getTagsSince(double)}
     */
    public static double getLastTime() {
        return lastTime;
    }

    /**
     * Returns the result of {@link #getTags()} if the result has changed since {@code time}, otherwise an empty list.
     * The time is in epoch seconds, the same units as {@link #getLastTime()}
     */
    public static List<Tag> getTagsSince(double time) {
        List<Tag> tags = getTags();
        return tags;
        // TODO reinstate once time networktable entry is implemented
        // return time < lastTime ? tags : List.of();
    }

}
