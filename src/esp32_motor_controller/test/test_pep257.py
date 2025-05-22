import os
import unittest
import pep257

class TestPep257Compliance(unittest.TestCase):
    def setUp(self):
        self.package_dir = os.path.dirname(os.path.abspath(__file__)) + '/../..'
        self.violations = []

    def test_pep257_compliance(self):
        checker = pep257.Checker(self.package_dir)
        self.violations = checker.check()
        self.assertEqual(len(self.violations), 0, f"PEP 257 violations found: {self.violations}")

if __name__ == '__main__':
    unittest.main()