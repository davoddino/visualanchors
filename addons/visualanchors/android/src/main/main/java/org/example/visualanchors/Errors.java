package org.example.visualanchors;

/**
 * Centralised error hierarchy for the Visual Anchors pipeline.
 * All failures are surfaced to Godot as consistent IllegalState/Argument exceptions.
 */
final class Errors {
    private Errors() {}

    static class ConfigurationMissing extends IllegalStateException {
        ConfigurationMissing(String message) {
            super(message);
        }
    }

    static class NotInitialized extends IllegalStateException {
        NotInitialized(String message) {
            super(message);
        }
    }

    static class InvalidInput extends IllegalArgumentException {
        InvalidInput(String message) {
            super(message);
        }
    }

    static class PoseComputationFailed extends RuntimeException {
        PoseComputationFailed(String message) {
            super(message);
        }

        PoseComputationFailed(String message, Throwable cause) {
            super(message, cause);
        }
    }
}
