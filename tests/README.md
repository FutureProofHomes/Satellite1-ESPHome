# Host tests

`openai_realtime/test_rt_util.cpp` covers the OpenAI Realtime component's pure helpers (URL
derivation, JSON scanning, base64, the 16->24 kHz resampler, /models parsing). Build and run:

```
g++ -std=c++17 -O2 -Wall -I esphome/components/openai_realtime \
    tests/openai_realtime/test_rt_util.cpp esphome/components/openai_realtime/rt_util.cpp -o /tmp/t && /tmp/t
```

`openai_realtime/test_rt_messages.cpp` builds every JSON message the component sends or serves (the
`fill_*` functions in `rt_messages.cpp`, which the device runs inside `esphome::json::build_json`)
and parses each one back. It needs ArduinoJson 7.4.3 (`git clone --depth 1 --branch v7.4.3
https://github.com/bblanchon/ArduinoJson.git /tmp/ArduinoJson`):

```
g++ -std=c++17 -O2 -Wall -I esphome/components/openai_realtime -I /tmp/ArduinoJson/src \
    tests/openai_realtime/test_rt_messages.cpp esphome/components/openai_realtime/rt_messages.cpp \
    esphome/components/openai_realtime/rt_util.cpp -o /tmp/tm && /tmp/tm
/tmp/tm --dump | python3 -c 'import json, sys; [json.loads(l) for l in sys.stdin]'
```
