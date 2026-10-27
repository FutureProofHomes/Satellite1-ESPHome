"""Compile production volume-control bodies against small host-only collaborators.

Run: python3 -B tests/volume_mute/test_volume_mute.py
Requires an existing C++17 compiler; acquires no dependencies.
Temporary outputs use Python's normal tempfile directory (honoring TMPDIR).
"""

from pathlib import Path
import os
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
COMPONENTS = ROOT / "esphome/components"


def method(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def main():
    speaker = (COMPONENTS / "i2s_audio/speaker/i2s_audio_speaker.cpp").read_text()
    player = (COMPONENTS / "speaker_source/speaker_source_media_player.cpp").read_text()
    proxy = (COMPONENTS / "satellite1/audio_dac/dac_proxy.cpp").read_text()
    proxy_header = (COMPONENTS / "satellite1/audio_dac/dac_proxy.h").read_text()
    provider = (COMPONENTS / "satellite1/audio_dac/volume_state.cpp").read_text()
    provider_header = (COMPONENTS / "satellite1/audio_dac/volume_state.h").read_text()
    state = (COMPONENTS / "speaker_source/volume_state.h").read_text()
    table_start = speaker.index("static const std::vector<int16_t> Q15_VOLUME_SCALING_FACTORS")
    table_end = speaker.index(";", table_start) + 1
    bodies = speaker[table_start:table_end] + "\n"
    constant_start = proxy.index("static constexpr uint8_t DAC_WRITE_ATTEMPTS")
    bodies += proxy[constant_start:proxy.index(";", constant_start) + 1] + "\n"
    for name in ("process_volume_state_", "update_software_volume_", "set_volume", "set_mute_state"):
        bodies += method(speaker, f"void I2SAudioSpeaker::{name}(") + "\n"
    for name in ("handle_volume_state_", "queue_volume_notifications_"):
        bodies += method(player, f"void SpeakerSourceMediaPlayer::{name}(") + "\n"
    for name in ("apply_mute_", "reconcile_", "receiving_current_output_", "set_mute_on", "set_mute_off", "set_volume", "request_output"):
        bodies += method(proxy, f"bool DACProxy::{name}(") + "\n"
    for name in ("notify_requested_output_", "select_output_", "set_audio_ready"):
        bodies += method(proxy, f"void DACProxy::{name}(") + "\n"
    for name, result in (("initialize", "void"), ("publish_selected_state_", "void"),
                         ("save_", "void"), ("set_volume", "bool"), ("set_muted", "bool")):
        bodies += method(provider, f"{result} DACOutputVolumeState::{name}(") + "\n"
    state = state[state.index("struct VolumeMuteState"):state.rindex("}  // namespace")]
    harness = Path(__file__).with_name("volume_mute_host.cpp").read_text()
    source = harness.replace("// PRODUCTION_STATE", state).replace("// PRODUCTION_METHODS", bodies)
    declarations = provider_header[provider_header.index("struct DACOutputVolumePreferences"):
                                   provider_header.index("}  // namespace satellite1")]
    source = source.replace("// PRODUCTION_PROVIDER", declarations)
    accessors = "\n".join(method(proxy_header, signature) for signature in (
        "bool audio_ready() const", "bool has_requested_output_notification() const",
        "DacOutput requested_output() const", "void set_volume_state("))
    source = source.replace("// PRODUCTION_PROXY_ACCESSORS", accessors)
    # Keep all generated sources/binaries outside the checkout and remove on exit.
    with tempfile.TemporaryDirectory(prefix="volume-mute-host-") as directory:
        cpp = Path(directory) / "host.cpp"
        cpp.write_text(source)
        for dac in (False, True):
            binary = Path(directory) / ("dac" if dac else "software")
            command = [os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                       "-fsanitize=address,undefined", "-g", str(cpp), "-o", str(binary)]
            if dac:
                command.append("-DUSE_AUDIO_DAC")
            subprocess.run(command, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
