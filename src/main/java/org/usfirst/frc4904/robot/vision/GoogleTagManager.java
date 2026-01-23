package org.usfirst.frc4904.robot.vision;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

/** Manages Google tags */
public class GoogleTagManager {

    private final NetworkTableEntry tagsEntry;
    private final ObjectMapper mapper = new ObjectMapper();

    public record Tag(int id, Rotation2d rot, Translation3d pos, int camera) {}

    public GoogleTagManager() {
        tagsEntry = NetworkTableInstance.getDefault().getEntry("/dauntless/tags");
    }

    public List<Tag> getTags() {
        List<Tag> tags = new ArrayList<>();

        String json = tagsEntry.getString("");
        if (json.isEmpty()) return tags;

        try {
            JsonNode root = mapper.readTree(json);

            for (JsonNode el : root) {
                JsonNode idPath = el.path("id");
                if (idPath.isNull()) continue;

                double[] pos = mapper.treeToValue(el.path("pos"), double[].class);

                tags.add(new Tag(
                    idPath.asInt(),
                    Rotation2d.fromRotations(el.path("rot").asDouble()),
                    new Translation3d(pos[2], pos[0], pos[1]),
                    0
                ));
            }
        } catch (Exception e) {
            System.out.println("google tag manager parsing error!!!");
        }

        return tags;
    }
}
