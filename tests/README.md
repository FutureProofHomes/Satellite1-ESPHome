# Host tests

Tests for the pure parts of `esphome/components/openai_realtime`, built and run on the host.

`openai_realtime/test_rt_util.cpp` covers URL derivation, JSON scanning, base64, the 16->24 kHz
resampler and wire-format detection:

```
g++ -std=c++17 -O2 -Wall -I esphome/components/openai_realtime \
    tests/openai_realtime/test_rt_util.cpp esphome/components/openai_realtime/rt_util.cpp -o /tmp/t && /tmp/t
```

`openai_realtime/test_rt_messages.cpp` builds every JSON message the component sends (the `fill_*`
functions in `rt_messages.cpp`, which the device runs inside `esphome::json::build_json`) and parses
each one back. It needs ArduinoJson 7.4.3, the version ESPHome 2026.9 pins:

```
git clone --depth 1 --branch v7.4.3 https://github.com/bblanchon/ArduinoJson.git /tmp/ArduinoJson
g++ -std=c++17 -O2 -Wall -I esphome/components/openai_realtime -I /tmp/ArduinoJson/src \
    tests/openai_realtime/test_rt_messages.cpp esphome/components/openai_realtime/rt_messages.cpp \
    esphome/components/openai_realtime/rt_util.cpp -o /tmp/tm && /tmp/tm
/tmp/tm --dump | python3 -c 'import json, sys; [json.loads(l) for l in sys.stdin]'
```
