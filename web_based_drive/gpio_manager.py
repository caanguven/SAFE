# gpio_manager.py

import RPi.GPIO as GPIO
import logging
import threading

class GPIOManager:
    _instance = None
    _lock = threading.Lock()
    _initialized = False

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(GPIOManager, cls).__new__(cls)
            return cls._instance

    def __init__(self):
        if not self._initialized:
            with self._lock:
                if not self._initialized:
                    self._setup_gpio()
                    self._initialized = True

    def _setup_gpio(self):
        """Initialize GPIO with proper mode and error handling."""
        try:
            # Clean up any existing GPIO settings
            GPIO.cleanup()
            
            # Set mode to BOARD
            GPIO.setmode(GPIO.BOARD)
            logging.info("GPIO mode set to BOARD")
            
        except Exception as e:
            logging.error(f"Error initializing GPIO: {e}")
            raise

    @staticmethod
    def get_mode():
        """Get current GPIO mode."""
        return GPIO.getmode()

    def cleanup(self):
        """Clean up GPIO settings."""
        with self._lock:
            if self._initialized:
                GPIO.cleanup()
                self._initialized = False
                logging.info("GPIO cleaned up")