import os
import subprocess

def test_flake8():
    """Run flake8 to check code style."""
    result = subprocess.run(['flake8', 'esp32_motor_controller'], capture_output=True, text=True)
    assert result.returncode == 0, f"Flake8 found issues:\n{result.stdout}\n{result.stderr}"

if __name__ == "__main__":
    test_flake8()