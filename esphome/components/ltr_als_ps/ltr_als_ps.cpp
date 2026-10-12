#include "ltr_als_ps.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <limits>

using esphome::i2c::ErrorCode;

namespace esphome::ltr_als_ps {

static const char *const TAG = "ltr_als_ps";

static const uint8_t MAX_TRIES = 5;
static const AlsGain GAINS[GAINS_COUNT] = {GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_48, GAIN_96};
static const IntegrationTime INT_TIMES[TIMES_COUNT] = {
    INTEGRATION_TIME_50MS,  INTEGRATION_TIME_100MS, INTEGRATION_TIME_150MS, INTEGRATION_TIME_200MS,
    INTEGRATION_TIME_250MS, INTEGRATION_TIME_300MS, INTEGRATION_TIME_350MS, INTEGRATION_TIME_400MS};

template<typename T, size_t size> T get_next(const T (&array)[size], const T val) {
  size_t i = 0;
  size_t idx = std::numeric_limits<size_t>::max();
  while (idx == std::numeric_limits<size_t>::max() && i < size) {
    if (array[i] == val) {
      idx = i;
      break;
    }
    i++;
  }
  if (idx == std::numeric_limits<size_t>::max() || i + 1 >= size)
    return val;
  return array[i + 1];
}

template<typename T, size_t size> T get_prev(const T (&array)[size], const T val) {
  size_t i = size - 1;
  size_t idx = std::numeric_limits<size_t>::max();
  while (idx == std::numeric_limits<size_t>::max() && i > 0) {
    if (array[i] == val) {
      idx = i;
      break;
    }
    i--;
  }
  if (idx == std::numeric_limits<size_t>::max() || i == 0)
    return val;
  return array[i - 1];
}

static uint16_t get_itime_ms(IntegrationTime time) {
  static const uint16_t ALS_INT_TIME[8] = {100, 50, 200, 400, 150, 250, 300, 350};
  return ALS_INT_TIME[time & 0b111];
}

static uint16_t get_meas_time_ms(MeasurementRepeatRate rate) {
  static const uint16_t ALS_MEAS_RATE[8] = {50, 100, 200, 500, 1000, 2000, 2000, 2000};
  return ALS_MEAS_RATE[rate & 0b111];
}

static float get_gain_coeff(AlsGain gain) {
  static const float ALS_GAIN[8] = {1, 2, 4, 8, 0, 0, 48, 96};
  return ALS_GAIN[gain & 0b111];
}

static float get_ps_gain_coeff(PsGain gain) {
  static const float PS_GAIN[4] = {16, 0, 32, 64};
  return PS_GAIN[gain & 0b11];
}

void LTRAlsPsComponent::setup() {
  // As per datasheet we need to wait at least 100ms after power on to get ALS chip responsive
  this->set_timeout(100, [this]() { this->state_ = State::DELAYED_SETUP; });
}

void LTRAlsPsComponent::dump_config() {
  auto get_device_type = [](LtrType typ) {
    switch (typ) {
      case LtrType::LTR_TYPE_ALS_ONLY:
        return "ALS only";
      case LtrType::LTR_TYPE_PS_ONLY:
        return "PS only";
      case LtrType::LTR_TYPE_ALS_AND_PS:
        return "ALS + PS";
      default:
        return "Unknown";
    }
  };

  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Device type: %s", get_device_type(this->ltr_type_));
  if (this->is_als_()) {
    ESP_LOGCONFIG(TAG,
                  "  Automatic mode: %s\n"
                  "  Gain: %.0fx\n"
                  "  Integration time: %d ms\n"
                  "  Measurement repeat rate: %d ms\n"
                  "  Glass attenuation factor: %f",
                  ONOFF(this->automatic_mode_enabled_), get_gain_coeff(this->gain_),
                  get_itime_ms(this->integration_time_), get_meas_time_ms(this->repeat_rate_),
                  this->glass_attenuation_factor_);
    LOG_SENSOR("  ", "ALS calculated lux", this->ambient_light_sensor_);
    LOG_SENSOR("  ", "CH1 Infrared counts", this->infrared_counts_sensor_);
    LOG_SENSOR("  ", "CH0 Visible+IR counts", this->full_spectrum_counts_sensor_);
    LOG_SENSOR("  ", "Actual gain", this->actual_gain_sensor_);
  }
  if (this->is_ps_()) {
    ESP_LOGCONFIG(TAG,
                  "  Proximity gain: %.0fx\n"
                  "  Proximity cooldown time: %d s\n"
                  "  Proximity high threshold: %d\n"
                  "  Proximity low threshold: %d",
                  get_ps_gain_coeff(this->ps_gain_), this->ps_cooldown_time_s_, this->ps_threshold_high_,
                  this->ps_threshold_low_);
    LOG_SENSOR("  ", "Proximity counts", this->proximity_counts_sensor_);
  }
  LOG_UPDATE_INTERVAL(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
}

void LTRAlsPsComponent::update() {
  ESP_LOGV(TAG, "Updating");
  if (this->is_ready() && this->state_ == State::IDLE) {
    ESP_LOGV(TAG, "Initiating new data collection");

    this->als_readings_.ch0 = 0;
    this->als_readings_.ch1 = 0;
    this->als_readings_.gain = this->gain_;
    this->als_readings_.integration_time = this->integration_time_;
    this->als_readings_.lux = 0;
    this->als_readings_.number_of_adjustments = 0;
    if (!this->is_als_()) {
      this->state_ = State::READY_TO_PUBLISH;
    } else if (!this->als_configuration_valid_) {
      this->state_ = State::COLLECTING_DATA_AUTO;
    } else {
      this->start_als_wait_(false);
    }
  } else {
    ESP_LOGV(TAG, "Component not ready yet");
  }
}

void LTRAlsPsComponent::loop() {
  ErrorCode err = i2c::ERROR_OK;

  switch (this->state_) {
    case State::DELAYED_SETUP:
      err = this->write(nullptr, 0);
      if (err != i2c::ERROR_OK) {
        ESP_LOGV(TAG, "i2c connection failed");
        this->mark_failed();
        return;
      }
      this->configure_reset_();
      if (this->is_als_()) {
        if (!this->configure_als_()) {
          ESP_LOGE(TAG, "Failed to activate ALS with configured gain");
          this->mark_failed();
          return;
        }
        if (!this->configure_integration_time_(this->integration_time_)) {
          ESP_LOGE(TAG, "Failed to configure ALS measurement timing");
          this->mark_failed();
          return;
        }
      }
      if (this->is_ps_()) {
        this->configure_ps_();
      }

      this->state_ = State::IDLE;
      if (this->is_als_()) {
        this->als_readings_.gain = this->gain_;
        this->als_readings_.integration_time = this->integration_time_;
        this->start_als_wait_(true);
      }
      break;

    case State::IDLE:
      if (this->is_ps_()) {
        check_and_trigger_ps_();
      }
      break;

    case State::WAITING_FOR_DATA: {
      LtrDataAvail availability = this->is_als_data_ready_(this->als_readings_);
      if (availability == LtrDataAvail::LTR_DATA_OK) {
        ESP_LOGV(TAG, "Reading sensor data having gain = %.0fx, time = %d ms", get_gain_coeff(this->als_readings_.gain),
                 get_itime_ms(this->als_readings_.integration_time));
        if (this->read_sensor_data_(this->als_readings_)) {
          this->state_ = State::DATA_COLLECTED;
          break;
        }
        availability = LtrDataAvail::LTR_IO_ERROR;
      }
      const uint32_t elapsed = millis() - this->als_wait_started_ms_;
      if (elapsed >= this->als_wait_timeout_ms_) {
        // Invalid readings can prevent count-based ranging after a bright transition at high sensitivity.
        if (availability == LtrDataAvail::LTR_BAD_DATA && this->automatic_mode_enabled_ &&
            this->als_readings_.number_of_adjustments < 16 && this->decrease_sensitivity_(this->als_readings_)) {
          ESP_LOGD(TAG, "Invalid ALS data at readiness deadline; reducing automatic sensitivity");
          this->state_ = State::COLLECTING_DATA_AUTO;
          break;
        }
        switch (availability) {
          case LtrDataAvail::LTR_BAD_DATA:
            this->als_warning_(LOG_STR("Timed out waiting for valid ALS data"));
            break;
          case LtrDataAvail::LTR_STALE_GAIN:
            this->als_warning_(LOG_STR("Timed out waiting for ALS data with requested gain"));
            break;
          case LtrDataAvail::LTR_IO_ERROR:
            this->als_warning_(LOG_STR("Timed out communicating with ALS sensor"));
            break;
          default:
            this->als_warning_(LOG_STR("Timed out waiting for new ALS data"));
            break;
        }
        this->state_ = State::IDLE;
      } else {
        this->state_ = State::ADJUSTMENT_IN_PROGRESS;
        const uint32_t wait = std::min(this->effective_als_period_ms_(this->als_readings_.integration_time),
                                       this->als_wait_timeout_ms_ - elapsed);
        this->set_timeout("als_ready", wait, [this]() { this->state_ = State::WAITING_FOR_DATA; });
      }
      break;
    }

    case State::COLLECTING_DATA_AUTO:
    case State::DATA_COLLECTED:
      // Reconfigure only changed settings; keep the settled range between updates.
      if (this->state_ == State::COLLECTING_DATA_AUTO || this->are_adjustments_required_(this->als_readings_)) {
        if (this->als_readings_.number_of_adjustments > 16) {
          this->als_warning_(LOG_STR("Too many ALS sensitivity adjustments; abandoning update"));
          this->state_ = State::IDLE;
          break;
        }
        ESP_LOGD(TAG, "Reconfiguring sensitivity: gain = %.0fx, time = %d ms", get_gain_coeff(this->als_readings_.gain),
                 get_itime_ms(this->als_readings_.integration_time));
        const bool force = !this->als_configuration_valid_;
        this->als_configuration_valid_ = false;
        if (force || this->integration_time_ != this->als_readings_.integration_time) {
          if (!this->configure_integration_time_(this->als_readings_.integration_time)) {
            this->als_warning_(LOG_STR("Failed to configure ALS integration time"));
            this->state_ = State::IDLE;
            break;
          }
          this->integration_time_ = this->als_readings_.integration_time;
        }
        if (force || this->gain_ != this->als_readings_.gain) {
          if (!this->configure_gain_(this->als_readings_.gain)) {
            this->als_warning_(LOG_STR("Failed to configure ALS gain"));
            this->state_ = State::IDLE;
            break;
          }
          this->gain_ = this->als_readings_.gain;
        }
        this->als_configuration_valid_ = true;
        this->start_als_wait_(true);
      } else {
        this->apply_lux_calculation_(this->als_readings_);
        this->state_ = State::READY_TO_PUBLISH;
      }
      break;

    case State::ADJUSTMENT_IN_PROGRESS:
      // nothing to be done, just waiting for the timeout
      break;

    case State::READY_TO_PUBLISH:
      this->publish_data_part_1_(this->als_readings_);
      this->state_ = State::KEEP_PUBLISHING;
      break;

    case State::KEEP_PUBLISHING:
      this->publish_data_part_2_(this->als_readings_);
      if (this->als_warning_active_ && !std::isnan(this->als_readings_.lux)) {
        this->status_clear_warning();
        this->als_warning_active_ = false;
        this->als_warning_message_ = nullptr;
      }
      this->state_ = State::IDLE;
      break;

    default:
      break;
  }
}

uint32_t LTRAlsPsComponent::effective_als_period_ms_(IntegrationTime time) const {
  return std::max(get_meas_time_ms(this->repeat_rate_), get_itime_ms(time));
}

void LTRAlsPsComponent::start_als_wait_(bool settling) {
  const uint32_t period = this->effective_als_period_ms_(this->als_readings_.integration_time);
  this->als_wait_started_ms_ = millis();
  // Allow two conversion periods for readiness, plus two to flush the old configuration when settling.
  this->als_wait_timeout_ms_ = (settling ? 4 : 2) * period + 50;
  if (settling) {
    this->state_ = State::ADJUSTMENT_IN_PROGRESS;
    this->set_timeout("als_ready", 2 * period + 20, [this]() { this->state_ = State::WAITING_FOR_DATA; });
  } else {
    this->state_ = State::WAITING_FOR_DATA;
  }
}

void LTRAlsPsComponent::als_warning_(const LogString *message) {
  if (this->als_warning_message_ != message) {
    ESP_LOGW(TAG, "%s", LOG_STR_ARG(message));
    this->als_warning_message_ = message;
  }
  if (!this->als_warning_active_) {
    this->status_set_warning(message);
    this->als_warning_active_ = true;
  }
}

void LTRAlsPsComponent::check_and_trigger_ps_() {
  uint16_t ps_data = this->read_ps_data_();
  uint32_t now = millis();

  if (ps_data != this->ps_readings_) {
    this->ps_readings_ = ps_data;
    // Higher values - object is closer to sensor
    if (ps_data > this->ps_threshold_high_ &&
        now - this->last_ps_high_trigger_time_ >= this->ps_cooldown_time_s_ * 1000) {
      this->last_ps_high_trigger_time_ = now;
      ESP_LOGV(TAG, "Proximity high threshold triggered. Value = %d, Trigger level = %d", ps_data,
               this->ps_threshold_high_);
      this->on_ps_high_trigger_callback_.call();
    } else if (ps_data < this->ps_threshold_low_ &&
               now - this->last_ps_low_trigger_time_ >= this->ps_cooldown_time_s_ * 1000) {
      this->last_ps_low_trigger_time_ = now;
      ESP_LOGV(TAG, "Proximity low threshold triggered. Value = %d, Trigger level = %d", ps_data,
               this->ps_threshold_low_);
      this->on_ps_low_trigger_callback_.call();
    }
  }
}

bool LTRAlsPsComponent::check_part_number_() {
  uint8_t manuf_id = this->reg((uint8_t) CommandRegisters::MANUFAC_ID).get();
  if (manuf_id != 0x05) {  // 0x05 is Lite-On Semiconductor Corp. ID
    ESP_LOGW(TAG, "Unknown manufacturer ID: 0x%02X", manuf_id);
    this->mark_failed();
    return false;
  }

  // Things getting not really funny here, we can't identify device type by part number ID
  // ======================== ========= ===== =================
  // Device                    Part ID   Rev   Capabilities
  // ======================== ========= ===== =================
  // Ltr-329/ltr-303            0x0a    0x00  Als 16b
  // Ltr-553/ltr-556/ltr-556    0x09    0x02  Als 16b + Ps 11b  diff nm sens
  // Ltr-659                    0x09    0x02  Ps 11b and ps gain
  //
  // There are other devices which might potentially work with default settings,
  // but registers layout is different and we can't use them properly. For ex. ltr-558

  PartIdRegister part_id{0};
  part_id.raw = this->reg((uint8_t) CommandRegisters::PART_ID).get();
  if (part_id.part_number_id != 0x0a && part_id.part_number_id != 0x09) {
    ESP_LOGW(TAG, "Unknown part number ID: 0x%02X. It might not work properly.", part_id.part_number_id);
    this->status_set_warning();
    return true;
  }
  return true;
}

void LTRAlsPsComponent::configure_reset_() {
  ESP_LOGV(TAG, "Resetting");

  AlsControlRegister als_ctrl{0};
  als_ctrl.sw_reset = true;
  this->reg((uint8_t) CommandRegisters::ALS_CONTR) = als_ctrl.raw;
  delay(2);

  uint8_t tries = MAX_TRIES;
  do {
    ESP_LOGV(TAG, "Waiting for chip to reset");
    delay(2);
    als_ctrl.raw = this->reg((uint8_t) CommandRegisters::ALS_CONTR).get();
  } while (als_ctrl.sw_reset && tries--);  // while sw reset bit is on - keep waiting

  if (als_ctrl.sw_reset) {
    ESP_LOGW(TAG, "Reset timed out");
  }
}

bool LTRAlsPsComponent::configure_als_() { return this->configure_gain_(this->gain_); }

void LTRAlsPsComponent::configure_ps_() {
  PsMeasurementRateRegister ps_meas{0};
  ps_meas.ps_measurement_rate = PsMeasurementRate::PS_MEAS_RATE_50MS;
  this->reg((uint8_t) CommandRegisters::PS_MEAS_RATE) = ps_meas.raw;

  PsControlRegister ps_ctrl{0};
  ps_ctrl.ps_mode_active = true;
  ps_ctrl.ps_mode_xxx = true;
  this->reg((uint8_t) CommandRegisters::PS_CONTR) = ps_ctrl.raw;
}

uint16_t LTRAlsPsComponent::read_ps_data_() {
  AlsPsStatusRegister als_status{0};
  als_status.raw = this->reg((uint8_t) CommandRegisters::ALS_PS_STATUS).get();
  if (!als_status.ps_new_data || als_status.data_invalid) {
    return this->ps_readings_;
  }

  uint8_t ps_low = this->reg((uint8_t) CommandRegisters::PS_DATA_0).get();
  PsData1Register ps_high;
  ps_high.raw = this->reg((uint8_t) CommandRegisters::PS_DATA_1).get();

  uint16_t val = encode_uint16(ps_high.ps_data_high, ps_low);
  if (ps_high.ps_saturation_flag) {
    return 0x7ff;  // full 11 bit range
  }
  return val;
}

bool LTRAlsPsComponent::configure_gain_(AlsGain gain) {
  AlsControlRegister als_ctrl{0};
  als_ctrl.active_mode = true;
  als_ctrl.gain = gain;
  AlsControlRegister read_als_ctrl{0};
  return this->write_byte((uint8_t) CommandRegisters::ALS_CONTR, als_ctrl.raw) &&
         this->read_byte((uint8_t) CommandRegisters::ALS_CONTR, &read_als_ctrl.raw) && read_als_ctrl.gain == gain &&
         read_als_ctrl.active_mode;
}

bool LTRAlsPsComponent::configure_integration_time_(IntegrationTime time) {
  MeasurementRateRegister meas{0};
  meas.measurement_repeat_rate = this->repeat_rate_;
  meas.integration_time = time;
  MeasurementRateRegister read_meas{0};
  return this->write_byte((uint8_t) CommandRegisters::MEAS_RATE, meas.raw) &&
         this->read_byte((uint8_t) CommandRegisters::MEAS_RATE, &read_meas.raw) && read_meas.integration_time == time &&
         read_meas.measurement_repeat_rate == this->repeat_rate_;
}

LtrDataAvail LTRAlsPsComponent::is_als_data_ready_(AlsReadings &data) {
  AlsPsStatusRegister als_status{0};
  // A failed final-byte read can leave the latch locked even when new-data status is clear.
  if (this->als_latch_release_pending_) {
    uint8_t discarded;
    if (!this->read_byte((uint8_t) CommandRegisters::ALS_DATA_CH0_1, &discarded)) {
      return LtrDataAvail::LTR_IO_ERROR;
    }
    this->als_latch_release_pending_ = false;
    return LtrDataAvail::LTR_NO_DATA;
  }
  if (!this->read_byte((uint8_t) CommandRegisters::ALS_PS_STATUS, &als_status.raw)) {
    return LtrDataAvail::LTR_IO_ERROR;
  }
  if (!als_status.als_new_data)
    return LtrDataAvail::LTR_NO_DATA;

  ESP_LOGV(TAG, "Data ready, reported gain is %.0f", get_gain_coeff(als_status.gain));
  if (data.gain != als_status.gain) {
    ESP_LOGV(TAG, "Ignoring ALS data from previous gain (requested %.0f)", get_gain_coeff(data.gain));
    return LtrDataAvail::LTR_STALE_GAIN;
  }
  if (als_status.data_invalid) {
    ESP_LOGV(TAG, "Waiting for valid ALS data");
    return LtrDataAvail::LTR_BAD_DATA;
  }
  return LtrDataAvail::LTR_DATA_OK;
}

bool LTRAlsPsComponent::read_sensor_data_(AlsReadings &data) {
  uint8_t bytes[4]{};
  bool success = true;
  // Complete the documented order even after an error so the final byte releases the data latch.
  for (uint8_t i = 0; i < 4; i++) {
    const bool byte_ok = this->read_byte((uint8_t) CommandRegisters::ALS_DATA_CH1_0 + i, &bytes[i]);
    if (i == 3)
      this->als_latch_release_pending_ = !byte_ok;
    if (!byte_ok)
      success = false;
  }
  if (!success) {
    return false;
  }
  data.ch1 = encode_uint16(bytes[1], bytes[0]);
  data.ch0 = encode_uint16(bytes[3], bytes[2]);

  ESP_LOGV(TAG, "Got sensor data: CH1 = %d, CH0 = %d", data.ch1, data.ch0);
  return true;
}

bool LTRAlsPsComponent::decrease_sensitivity_(AlsReadings &data) {
  const AlsGain prev_gain = get_prev(GAINS, data.gain);
  if (prev_gain != data.gain) {
    data.gain = prev_gain;
    data.number_of_adjustments++;
    return true;
  }
  const IntegrationTime prev_time = get_prev(INT_TIMES, data.integration_time);
  if (prev_time != data.integration_time) {
    data.integration_time = prev_time;
    data.number_of_adjustments++;
    return true;
  }
  return false;
}

bool LTRAlsPsComponent::are_adjustments_required_(AlsReadings &data) {
  if (!this->automatic_mode_enabled_)
    return false;

  // Recommended thresholds as per datasheet
  static const uint16_t LOW_INTENSITY_THRESHOLD = 1000;
  static const uint16_t HIGH_INTENSITY_THRESHOLD = 30000;

  // Use both channels so IR-heavy light cannot repeatedly increase gain into CH1 saturation.
  const uint16_t intensity = std::max(data.ch0, data.ch1);
  if (intensity >= HIGH_INTENSITY_THRESHOLD) {
    if (this->decrease_sensitivity_(data)) {
      ESP_LOGV(TAG, "High illuminance. Decreasing sensitivity.");
      return true;
    }
  } else if (intensity <= LOW_INTENSITY_THRESHOLD) {
    AlsGain next_gain = get_next(GAINS, data.gain);
    if (next_gain != data.gain) {
      data.gain = next_gain;
      data.number_of_adjustments++;
      ESP_LOGV(TAG, "Low illuminance. Increasing gain.");
      return true;
    }
    IntegrationTime next_time = get_next(INT_TIMES, data.integration_time);
    // The IC clamps integration above the repeat period; do not normalize lux using an unachievable time.
    if (next_time != data.integration_time && get_itime_ms(next_time) <= get_meas_time_ms(this->repeat_rate_)) {
      data.integration_time = next_time;
      data.number_of_adjustments++;
      ESP_LOGV(TAG, "Low illuminance. Increasing integration time.");
      return true;
    }
  } else {
    ESP_LOGD(TAG, "Illuminance is sufficient.");
    return false;
  }
  ESP_LOGD(TAG, "Can't adjust sensitivity anymore.");
  return false;
}

void LTRAlsPsComponent::apply_lux_calculation_(AlsReadings &data) {
  if ((data.ch0 == 0xFFFF) || (data.ch1 == 0xFFFF)) {
    this->als_warning_(LOG_STR("ALS channels saturated at current range"));
    data.lux = NAN;
    return;
  }

  const float ch0 = data.ch0;
  const float ch1 = data.ch1;
  const float total = ch0 + ch1;
  if (total <= 0.0f) {
    data.lux = 0.0f;
    return;
  }

  const float ratio = ch1 / total;
  float als_gain = get_gain_coeff(data.gain);
  float als_time = ((float) get_itime_ms(data.integration_time)) / 100.0f;
  float inv_pfactor = this->glass_attenuation_factor_;
  float lux = 0.0f;

  if (ratio < 0.45f) {
    lux = (1.7743f * ch0 + 1.1059f * ch1);
  } else if (ratio < 0.64f && ratio >= 0.45f) {
    lux = (4.2785f * ch0 - 1.9548f * ch1);
  } else if (ratio < 0.85f && ratio >= 0.64f) {
    lux = (0.5926f * ch0 + 0.1185f * ch1);
  } else {
    lux = 0.0f;
  }
  lux = inv_pfactor * lux / als_gain / als_time;
  data.lux = lux;

  ESP_LOGV(TAG, "Lux calculation: ratio %.3f, gain %.0fx, int time %.1f, inv_pfactor %.3f, lux %.3f", ratio, als_gain,
           als_time, inv_pfactor, lux);
}

void LTRAlsPsComponent::publish_data_part_1_(AlsReadings &data) {
  if (this->proximity_counts_sensor_ != nullptr) {
    this->proximity_counts_sensor_->publish_state(this->ps_readings_);
  }
  if (this->ambient_light_sensor_ != nullptr) {
    this->ambient_light_sensor_->publish_state(data.lux);
  }
  if (this->infrared_counts_sensor_ != nullptr) {
    this->infrared_counts_sensor_->publish_state(data.ch1);
  }
  if (this->full_spectrum_counts_sensor_ != nullptr) {
    this->full_spectrum_counts_sensor_->publish_state(data.ch0);
  }
}

void LTRAlsPsComponent::publish_data_part_2_(AlsReadings &data) {
  if (this->actual_gain_sensor_ != nullptr) {
    this->actual_gain_sensor_->publish_state(get_gain_coeff(data.gain));
  }
  if (this->actual_integration_time_sensor_ != nullptr) {
    this->actual_integration_time_sensor_->publish_state(get_itime_ms(data.integration_time));
  }
}
}  // namespace esphome::ltr_als_ps
