# Copyright compliance test for the esp32_motor_controller package

import unittest

class TestCopyrightCompliance(unittest.TestCase):
    def test_copyright_notice(self):
        # Check if the copyright notice is present in the source files
        source_files = [
            'esp32_motor_controller/motor_controller_node.py',
            'esp32_motor_controller/__init__.py',
            'esp32_motor_controller/config/motor_controller.yaml',
            'esp32_motor_controller/launch/motor_controller.launch.py',
            'esp32_motor_controller/test/test_flake8.py',
            'esp32_motor_controller/test/test_pep257.py',
            'esp32_motor_controller/README.md'
        ]
        
        for file in source_files:
            with open(file, 'r') as f:
                content = f.read()
                self.assertIn('Copyright', content, f'Copyright notice missing in {file}')

if __name__ == '__main__':
    unittest.main()