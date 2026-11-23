"""Direct STT wake verification without a Home Assistant custom integration."""

import esphome.codegen as cg

CODEOWNERS = ["@TaterTotterson"]
stt_wake_verifier_ns = cg.esphome_ns.namespace("stt_wake_verifier")
