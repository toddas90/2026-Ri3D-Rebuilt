package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.io.IOException;
import java.io.UncheckedIOException;

/**
 * Custom AprilTagFields implementation for 2026 Reefscape field
 */
public enum AprilTagFields {
    k2026RebuiltAndymark("2026-rebuilt-andymark.json");

    public final String resourceFile;

    AprilTagFields(String resourceFile) {
        this.resourceFile = resourceFile;
    }

    /**
     * Load the AprilTag field layout from the resource file
     * @return The loaded AprilTagFieldLayout
     * @throws UncheckedIOException if the resource file cannot be loaded
     */
    public AprilTagFieldLayout loadAprilTagLayoutField() {
        try {
            return AprilTagFieldLayout.loadFromResource(resourceFile);
        } catch (IOException e) {
            throw new UncheckedIOException(
                "Failed to load AprilTag field layout from " + resourceFile, e);
        }
    }
}