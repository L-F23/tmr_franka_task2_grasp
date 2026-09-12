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
            r"ARG ROS_IMAGE=ros:humble-ros-base-jammy@sha256:[0-9a-f]{64}",
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

        zed_bridge_branch = re.search(
            r"\n  zed-bridge\)\n(?P<body>.*?)\n    ;;",
            entrypoint,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(zed_bridge_branch)
        self.assertIn("ros2 run domain_bridge domain_bridge", zed_bridge_branch.group("body"))
        self.assertIn("zed_domain_bridge.yaml", zed_bridge_branch.group("body"))
        self.assertNotIn("--execute", zed_bridge_branch.group("body"))

    def test_operator_commands_require_no_development_host_directories(self):
        guide = (ROOT / "DOCKER.md").read_text(encoding="utf-8")
        self.assertNotIn("__PINNED_COMMIT__", guide)
        self.assertIn("<FULL_40_CHARACTER_PINNED_COMMIT_SHA>", guide)
        self.assertNotIn("src=/home/aup", guide)
        self.assertNotIn("src=/run/screen", guide)
        self.assertNotIn("src=/tmp/tmr_task2_motion.lock", guide)
        self.assertNotIn("SSH_AUTH_SOCK", guide)
        self.assertNotIn("TMR_SSH_IDENTITY_FILE", guide)
        self.assertNotIn("--privileged \\", guide)
        self.assertNotIn("--ipc host", guide)

    def test_custom_interfaces_and_camera_bridge_are_built_into_image(self):
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text(encoding="utf-8")
        self.assertIn("/opt/tmr-interfaces/install/setup.bash", entrypoint)
        self.assertIn("colcon build --merge-install", dockerfile)
        self.assertIn("franka_spine_msgs", dockerfile)
        self.assertIn("camera_viewer.py --port 18081", entrypoint)
        self.assertIn("ros2 run domain_bridge domain_bridge", entrypoint)
        self.assertIn("zed-bridge", entrypoint)
        self.assertIn("ros-humble-domain-bridge", dockerfile)
        self.assertTrue((ROOT / "docker" / "zed_domain_bridge.yaml").is_file())
        self.assertNotIn("stage_base_policy", entrypoint)
        self.assertIn("/opt/ros/humble/setup.bash", entrypoint)
        self.assertNotIn("/opt/ros/jazzy", entrypoint)
        self.assertNotIn("tmr_env.sh", entrypoint)
        self.assertNotIn("/run/screen", entrypoint)
        self.assertTrue((ROOT / "third_party" / "franka_ros2_interfaces" /
                         "franka_msgs" / "package.xml").is_file())
        self.assertTrue((ROOT / "third_party" / "franka_ros2_interfaces" /
                         "franka_spine_msgs" / "package.xml").is_file())

    def test_base_runtime_is_bundled_without_an_external_repository(self):
        source = (ROOT / "policy" / "base_runtime" /
                  "ensure_runtime.sh").read_text(encoding="utf-8")
        adapter = (ROOT / "policy" / "base_runtime" /
                   "cmd_vel_adapter.py").read_text(encoding="utf-8")
        self.assertIn("/swerve_drive_controller/odom", source)
        self.assertIn("/tmr_cycle/mission_cmd_vel", source)
        self.assertIn("/swerve_drive_controller/cmd_vel", adapter)
        self.assertNotIn("tmr-mobile-manipulation", source + adapter)

    def test_humble_runtime_uses_no_cross_version_dds_configuration(self):
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        guide = (ROOT / "DOCKER.md").read_text(encoding="utf-8")
        self.assertIn("RMW_IMPLEMENTATION=rmw_fastrtps_cpp", dockerfile)
        self.assertIn("FASTDDS_BUILTIN_TRANSPORTS=UDPv4", dockerfile)
        self.assertNotIn("CYCLONEDDS_URI", dockerfile)
        self.assertFalse((ROOT / "docker" / "cyclonedds.xml").exists())
        self.assertNotIn("ROS 2 Jazzy", guide)

    def test_operator_starts_isolated_zed_bridge_before_policy(self):
        guide = (ROOT / "DOCKER.md").read_text(encoding="utf-8")
        bridge_at = guide.index('"$POLICY_IMAGE" zed-bridge')
        check_at = guide.index('"$POLICY_IMAGE" check')
        self.assertLess(bridge_at, check_at)
        self.assertIn("ZED_DOMAIN_ID=1", guide)
        self.assertIn("TMR_MAIN_CAMERA_TOPIC=/tmr_task2/zed/image/compressed", guide)
        self.assertGreaterEqual(guide.count("FASTDDS_BUILTIN_TRANSPORTS=UDPv4"), 6)
        self.assertNotIn("TMR_MAIN_CAMERA_SOURCE=url", guide)

        workflow = (ROOT / ".github" / "workflows" / "docker.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn("Verify the isolated ZED DDS domain bridge", workflow)
        self.assertIn("TMR_ZED_DOMAIN_ID=1", workflow)
        self.assertIn("ROS_DOMAIN_ID=0", workflow)
        self.assertEqual(workflow.count("FASTDDS_BUILTIN_TRANSPORTS=UDPv4"), 3)


if __name__ == "__main__":
    unittest.main()
