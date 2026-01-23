package org.usfirst.frc4904.robot.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Manages Google tags */
public class GoogleTagManager {

    private final NetworkTable tagsTable;

    public record Tag(int id, Rotation2d rot, Translation3d pos, int camera) {}

    public GoogleTagManager() {
        tagsTable = NetworkTableInstance.getDefault()
            .getTable("dauntless")
            .getSubTable("tags");
    }

    public List<Tag> getTags() {
        List<Tag> tags = new ArrayList<>();

        for (String key : tagsTable.getSubTables()) {
            NetworkTable tagTable = tagsTable.getSubTable(key);

            int id = tagTable.getEntry("id").getNumber(-1).intValue();
            double rot = tagTable.getEntry("rot").getDouble(0.0);
            double[] pos = tagTable.getEntry("pos").getDoubleArray(new double[0]);

            tags.add(new Tag(
                id,
                Rotation2d.fromRotations(rot),
                new Translation3d(pos[2], pos[0], pos[1]),
                0
            ));
        }

        return tags;
    }
}
