package org.usfirst.frc4904.standard.custom.sensors;

import java.io.Serial;

public class InvalidSensorException extends Exception {
	@Serial
	private static final long serialVersionUID = -8211283541364995438L;

	public InvalidSensorException() {
		super();
	}

	public InvalidSensorException(String message) {
		super(message);
	}

	public InvalidSensorException(String message, Throwable cause) {
		super(message, cause);
	}

	public InvalidSensorException(Throwable cause) {
		super(cause);
	}
}
