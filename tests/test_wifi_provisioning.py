"""Actual YAML lifecycle and validated config-graph tests, not hardware tests."""

from pathlib import Path
import unittest

from esphome import config, yaml_util
from esphome.core import CORE


ROOT = Path(__file__).resolve().parents[1]
CONFLICT = "WiFi provisioning cannot be combined with YAML WiFi credentials."


class BootLifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = yaml_util.load_yaml(ROOT / "config/common/wifi_provisioning.yaml")
        cls.scripts = {item["id"]: item for item in cls.package["script"]}

    def test_ble_is_disabled_until_priority_200_boot_policy(self):
        self.assertIs(self.package["esp32_ble"]["enable_on_boot"], False)
        self.assertEqual(self.package["esphome"]["on_boot"], [{
            "priority": 200,
            "then": [{"script.execute": "wifi_boot_provisioning"}],
        }])
        boot = self.scripts["wifi_boot_provisioning"]["then"]
        self.assertEqual(boot, [
            {"if": {
                "condition": {"lambda": "return id(wifi_id).has_sta();"},
                "then": [{"delay": "90s"}],
            }},
            {"if": {
                "condition": {"lambda": "return !id(wifi_id).is_connected() && !id(wifi_ever_connected);"},
                "then": [{"ble.enable": None}],
            }},
        ])
        self.assertEqual(self.package["globals"], [{
            "id": "wifi_ever_connected", "type": "bool",
            "restore_value": False, "initial_value": "false",
        }])

    def test_first_connection_latches_and_cancels_initial_timer(self):
        self.assertEqual(self.package["wifi"]["on_connect"], [
            {"lambda": "id(wifi_ever_connected) = true;"},
            {"script.stop": "wifi_boot_provisioning"},
            {"script.execute": "wifi_finish_connection"},
        ])

    def test_disconnect_cancels_finish_and_disables_ble_only_after_connection(self):
        self.assertEqual(self.package["wifi"]["on_disconnect"], [
            {"script.stop": "wifi_finish_connection"},
            {"if": {
                "condition": {"lambda": "return id(wifi_ever_connected);"},
                "then": [{"ble.disable": None}],
            }},
        ])

    def test_finish_is_restartable_with_conditional_response_delay(self):
        finish = self.scripts["wifi_finish_connection"]
        self.assertEqual(finish["mode"], "restart")
        self.assertEqual(finish["then"], [
            {"if": {
                "condition": {"lambda": "return id(provisioning_ble).is_active();"},
                "then": [{"delay": "5s"}],
            }},
            {"ble.disable": None},
            {"wait_until": {"not": "ble.enabled"}},
            {"delay": "1s"},
        ])
        self.assertEqual(self.package["voice_assistant"]["on_client_connected"], [
            {"script.wait": "wifi_finish_connection"},
        ])

    def test_production_extends_finish_with_update(self):
        production = yaml_util.load_yaml(ROOT / "config/satellite1.yaml")
        self.assertEqual(production["script"][0]["then"], [
            {"component.update": "update_http_request"},
        ])
        self.assertEqual(str(production["script"][0]["id"]), "!extend wifi_finish_connection")

class ConfigGraphTest(unittest.TestCase):
    def load_config(self, path):
        CORE.reset()
        CORE.config_path = path
        return config.load_config({}, skip_external_update=True)

    def load_fixture(self, name):
        return self.load_config(ROOT / "tests/config" / name)

    def tearDown(self):
        CORE.reset()

    def test_default_dashboard_option_and_local_equivalent(self):
        dashboard = yaml_util.load_yaml(ROOT / "config/satellite1.dashboard.yaml")
        files = dashboard["packages"]["FutureProofHomes.Satellite1"]["files"]
        self.assertLess(files.index("config/common/wifi_provisioning.yaml"),
                        files.index("config/satellite1.base.yaml"))
        self.assertIn("config/common/dashboard_build.yaml", files)
        result = self.load_fixture("wifi_provisioning.yaml")
        self.assertFalse(result.errors, result.errors)
        for domain in ("wifi_provisioning", "esp32_ble", "esp32_improv", "improv_serial"):
            self.assertIn(domain, result)
        self.assertEqual(result["wifi_provisioning"], {})
        self.assertTrue(any(str(button["id"]) == "flash_embedded" for button in result["button"]))
        self.assertNotIn("update", result)

    def test_omitting_provisioning_keeps_credentials_without_improv(self):
        result = self.load_fixture("wifi_credentials.yaml")
        self.assertFalse(result.errors, result.errors)
        self.assertEqual(result["wifi"]["networks"][0]["ssid"], "Validation network")
        for domain in ("wifi_provisioning", "esp32_ble", "esp32_improv", "improv_serial"):
            self.assertNotIn(domain, result)
        self.assertTrue(any(str(button["id"]) == "flash_embedded" for button in result["button"]))
        self.assertNotIn("wifi_finish_connection", {str(item["id"]) for item in result["script"]})

    def test_normalized_credentials_rejected_for_shorthand_and_package_networks(self):
        for name in ("wifi_provisioning_invalid_ssid.yaml", "wifi_provisioning_invalid_networks.yaml"):
            with self.subTest(fixture=name):
                result = self.load_fixture(name)
                self.assertEqual(result["wifi"]["networks"][0]["ssid"], "Validation network")
                self.assertTrue(any(CONFLICT in error.msg for error in result.errors), result.errors)

    def test_production_keeps_wifi_enabled_and_xmos_flash_policy(self):
        production = self.load_config(ROOT / "config/satellite1.yaml")
        self.assertFalse(production.errors, production.errors)
        self.assertTrue(production["wifi"]["enable_on_boot"])
        self.assertEqual(production["substitutions"]["xmos_auto_flash_on_boot"], "true")

    def test_merged_disconnect_handler_keeps_boot_provisioning_available(self):
        for path in (ROOT / "config/satellite1.yaml", ROOT / "tests/config/wifi_provisioning.yaml"):
            with self.subTest(path=path.name):
                result = self.load_config(path)
                self.assertFalse(result.errors, result.errors)
                connected = next(item for item in result["globals"] if str(item["id"]) == "wifi_ever_connected")
                self.assertEqual(connected["initial_value"], "false")
                self.assertFalse(connected["restore_value"])

                actions = result["wifi"]["on_disconnect"]["then"]
                stops = [index for index, action in enumerate(actions)
                         if "script.stop" in action and str(action["script.stop"]["id"]) == "wifi_finish_connection"]
                self.assertEqual(len(stops), 1)
                guard = actions[stops[0] + 1]["if"]
                self.assertEqual(guard["condition"]["lambda"].value, "return id(wifi_ever_connected);")
                self.assertEqual([[key for key in action if key != "type_id"] for action in guard["then"]],
                                 [["ble.disable"]])
                self.assertFalse(any("ble.disable" in action or "ble.enable" in action for action in actions))


if __name__ == "__main__":
    unittest.main()
