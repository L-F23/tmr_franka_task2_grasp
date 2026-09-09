from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class DockerContractTests(unittest.TestCase):
    def test_repository_layout_keeps_runtime_under_policy(self):
        self.assertEqual(list(ROOT.glob("*.py")), [])
        self.assertTrue((ROOT / "policy" / "quick_start.py").is_file())
        self.assertTrue((ROOT / "policy" / "config").is_dir())
        self.assertTrue((ROOT / "policy" / "captures").is_dir())

        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("policy/                 Python policy", readme)
        self.assertIn("## Docker quick start", readme)
        self.assertIn("[Docker deployment](DOCKER.md#operator-commands)", readme)
        self.assertNotIn(
            "cd /home/aup/tmr_franka_task2_grasp &&",
            readme,
        )

    def test_base_image_is_digest_pinned_and_default_is_read_only_check(self):
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        self.assertRegex(
            dockerfile,
            r"ARG ROS_IMAGE=ros:jazzy-ros-base-noble@sha256:[0-9a-f]{64}",
        )
        self.assertIn(
            'ENTRYPOINT ["/usr/bin/tini", "--", '
            '"/opt/tmr-task2/docker/entrypoint.sh"]',
            dockerfile,
        )
        self.assertIn("POLICY_ROOT=/opt/tmr-task2/policy", dockerfile)
        self.assertIn("WORKDIR /opt/tmr-task2/policy", dockerfile)
        self.assertIn('CMD ["check"]', dockerfile)

    def test_physical_motion_requires_the_explicit_execute_mode(self):
        entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text(encoding="utf-8")
        execute_branch = re.search(
            r"\n  execute\)\n(?P<body>.*?)\n    ;;",
            entrypoint,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(execute_branch)
        self.assertIn(
            "exec /usr/bin/python3 -u quick_start.py --execute",
            execute_branch.group("body"),
        )
        self.assertNotIn("--execute", re.search(
            r"\n  check\)\n(?P<body>.*?)\n    ;;",
            entrypoint,
            flags=re.DOTALL,
        ).group("body"))

    def test_operator_commands_require_no_development_host_directories(self):
        guide = (ROOT / "DOCKER.md").read_text(encoding="utf-8")
        self.assertNotIn("__PINNED_COMMIT__", guide)
        self.assertIn("<FULL_40_CHARACTER_PINNED_COMMIT_SHA>", guide)
        self.assertNotIn("src=/home/aup", guide)
        self.assertNotIn("src=/run/screen", guide)
        self.assertNotIn("src=/tmp/tmr_task2_motion.lock", guide)
        self.assertNotIn("SSH_AUTH_SOCK", guide)
        self.assertNotIn("TMR_SSH_IDENTITY_FILE", guide)
        self.assertNotIn("ssh -", guide)
        self.assertNotIn("--privileged \\", guide)

    def test_custom_interfaces_and_camera_bridge_are_built_into_image(self):
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text(encoding="utf-8")
        self.assertIn("/opt/tmr-interfaces/install/setup.bash", entrypoint)
        self.assertIn("colcon build --merge-install", dockerfile)
        self.assertIn("franka_spine_msgs", dockerfile)
        self.assertIn("camera_viewer.py --port 18081", entrypoint)
        self.assertNotIn("tmr_env.sh", entrypoint)
        self.assertNotIn("/run/screen", entrypoint)
        self.assertTrue((ROOT / "third_party" / "franka_ros2_interfaces" /
                         "franka_msgs" / "package.xml").is_file())
        self.assertTrue((ROOT / "third_party" / "franka_ros2_interfaces" /
                         "franka_spine_msgs" / "package.xml").is_file())


if __name__ == "__main__":
    unittest.main()
