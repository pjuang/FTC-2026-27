package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Looks up a device that might not be in the Driver Hub robot configuration yet.
 *
 * <p>Returns {@code null} instead of throwing, so a robot that does not have an intake or a launcher
 * wired up can still run the Auto: the subsystem just does nothing, and the drive still LEAVEs and
 * PARKs.</p>
 */
public final class OptionalHardware {
    private OptionalHardware() {}

    public static <T extends HardwareDevice> T get(HardwareMap hardwareMap, Class<T> type, String name) {
        if (name == null || name.isEmpty()) return null;
        try {
            return hardwareMap.tryGet(type, name);
        } catch (RuntimeException e) {
            // Wrong device type under that name, or anything else: treat as missing.
            return null;
        }
    }
}
