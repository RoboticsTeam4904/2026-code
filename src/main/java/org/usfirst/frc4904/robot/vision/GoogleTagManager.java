package org.usfirst.frc4904.robot.vision;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Manages Google tags */
public class GoogleTagManager {
    private final HttpClient client;

    public record Tag(int id, Rotation2d rot, Translation3d pos, int camera) {}

    public GoogleTagManager() {
        client = HttpClient.newHttpClient();
    }

    public List<Tag> getTags() {
        HttpRequest request =
            HttpRequest.newBuilder()
                .uri(URI.create("http://10.49.4.203:8000/api/tags"))
                .GET()
                .build();

        List<Tag> tags = new ArrayList<>();

        String json;

        try {
            HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());
            json = response.body();
        } catch (IOException | InterruptedException e) {
            System.out.println("google tag manager fetching error!!!\n" + e.getClass().getName() + ": " + e.getMessage());
            return tags;
        }

        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(json);

            for (JsonNode el : root) {
                JsonNode idPath = el.path("id");
                if (idPath.isNull()) continue; // skip tags with no id

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
