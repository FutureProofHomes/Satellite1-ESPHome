#include "tas2780.h"

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tas2780 {

static const char *const TAG = "tas2780";

static const uint8_t TAS2780_PAGE_SELECT = 0x00;        // Page Select

/* PAGE 0*/
static const uint8_t TAS2780_MODE_CTRL = 0x02;        
static const uint8_t TAS2780_MODE_CTRL_BOP_SRC__PVDD_UVLO  = 0x80;
static const uint8_t TAS2780_MODE_CTRL_MODE_MASK  = 0x07;
static const uint8_t TAS2780_MODE_CTRL_MODE__ACTIVE  = 0x00;
static const uint8_t TAS2780_MODE_CTRL_MODE__ACTIVE_MUTED  = 0x01;
static const uint8_t TAS2780_MODE_CTRL_MODE__SFTW_SHTDWN  = 0x02;

static const uint8_t TAS2780_CHNL_0 = 0x03;
static const uint8_t TAS2780_CHNL_0_CDS_MODE_SHIFT = 6;
static const uint8_t TAS2780_CHNL_0_CDS_MODE_MASK = (0x03 << TAS2780_CHNL_0_CDS_MODE_SHIFT);
static const uint8_t TAS2780_CHNL_0_AMP_LEVEL_SHIFT = 1;
static const uint8_t TAS2780_CHNL_0_AMP_LEVEL_MASK = (0x1F) << TAS2780_CHNL_0_AMP_LEVEL_SHIFT;

static const uint8_t TAS2780_DC_BLK0 = 0x04;
static const uint8_t TAS2780_DC_BLK0_VBAT1S_MODE_SHIFT  = 7;

static const uint8_t TAS2780_DVC = 0x1a;

static const uint8_t TAS2780_INT_MASK0 = 0x3b;
static const uint8_t TAS2780_INT_MASK1 = 0x3c;
static const uint8_t TAS2780_INT_MASK4 = 0x3d;
static const uint8_t TAS2780_INT_MASK2 = 0x40;
static const uint8_t TAS2780_INT_MASK3 = 0x41;



/* PAGE 1*/
static const uint8_t TAS2780_INT_LDO = 0x36;

static const uint8_t POWER_MODES[4][2] = {
  {2, 0}, // PWR_MODE0: CDS_MODE=10, VBAT1S_MODE=0
  {0, 0}, // PWR_MODE1: CDS_MODE=00, VBAT1S_MODE=0
  {3, 1}, // PWR_MODE2: CDS_MODE=11, VBAT1S_MODE=1
  {1, 0}, // PWR_MODE3: CDS_MODE=01, VBAT1S_MODE=0
};


void TAS2780::setup(){
  // select page 0
  this->reg(TAS2780_PAGE_SELECT) = 0x00;
   
  // software reset
  this->reg(0x01) = 0x01;
  
  uint8_t chd1 = this->reg(0x05).get();
  uint8_t chd2 = this->reg(0x68).get();
  uint8_t chd3 = this->reg(0x02).get();

  if( chd1 == 0x41 ){
    ESP_LOGD(TAG, "TAS2780 chip found.");
    ESP_LOGD(TAG, "Reg 0x68: %d.", chd2 );
    ESP_LOGD(TAG, "Reg 0x02: %d.", chd3 );
  }
  else
  {
    ESP_LOGD(TAG, "TAS2780 chip not found.");
    this->mark_failed();
    return;
  }
  
  this->reg(TAS2780_PAGE_SELECT) = 0x00;
  this->reg(0x0e) = 0x44;
  this->reg(0x0f) = 0x40;

  this->reg(0x0a) = (2 << 4) | (3 << 2) | 2 ;

  this->reg(TAS2780_PAGE_SELECT) = 0x01;
  this->reg(0x17) = 0xc0;
  this->reg(0x19) = 0x00;
  this->reg(0x21) = 0x00;
  this->reg(0x35) = 0x74;

  this->reg(TAS2780_PAGE_SELECT) = 0xFD;
  this->reg(0x0d) = 0x0d;
  this->reg(0x3e) = 0x4a;
  this->reg(0x0d) = 0x00;


  this->reg(TAS2780_PAGE_SELECT) = 0x00;
  //Power Mode 2 (no external VBAT)
  //this->reg(0x03) = 0xe8;
  this->reg(0x03) = 0xa8;
  this->reg(0x04) = 0xa1;
  this->reg(0x71) = 0x12; 
  

  //Set interrupt masks
  this->reg(TAS2780_PAGE_SELECT) = 0x00;
  //mask VBAT1S Under Voltage
  this->reg(TAS2780_INT_MASK4) = 0xFF;
  //mask all PVDD and VBAT1S interrupts
  this->reg(TAS2780_INT_MASK2) = 0xFF;
  this->reg(TAS2780_INT_MASK3) = 0xFF;
  this->reg(TAS2780_INT_MASK1) = 0xFF;
  
  
  // set interrupt to trigger For 
  // 0h : On any unmasked live interrupts
  // 3h : 2 - 4 ms every 4 ms on any unmasked latched
  uint8_t reg_0x5c = this->reg(0x5c).get();
  this->reg(0x5c) = (reg_0x5c & ~0x03) | 0x00;   

  //set to software shutdown
  this->reg(TAS2780_MODE_CTRL) = (TAS2780_MODE_CTRL_BOP_SRC__PVDD_UVLO & ~TAS2780_MODE_CTRL_MODE_MASK) | TAS2780_MODE_CTRL_MODE__SFTW_SHTDWN;
  }


void TAS2780::activate(){
  ESP_LOGD(TAG, "Activating TAS2780");
  // clear interrupt latches
    this->reg(0x5c) = 0x19 | (1 << 2);
  // activate 
  this->reg(TAS2780_MODE_CTRL) = (TAS2780_MODE_CTRL_BOP_SRC__PVDD_UVLO & ~TAS2780_MODE_CTRL_MODE_MASK) | TAS2780_MODE_CTRL_MODE__ACTIVE;
}

void TAS2780::deactivate(){
  ESP_LOGD(TAG, "Dectivating TAS2780");
  //set to software shutdown
  this->reg(TAS2780_MODE_CTRL) = (TAS2780_MODE_CTRL_BOP_SRC__PVDD_UVLO & ~TAS2780_MODE_CTRL_MODE_MASK) | TAS2780_MODE_CTRL_MODE__SFTW_SHTDWN;
}


void TAS2780::reset(){
  // select page 0
  this->reg(TAS2780_PAGE_SELECT) = 0x00;
   
  // software reset
  this->reg(0x01) = 0x01;
  
  uint8_t chd1 = this->reg(0x05).get();
  uint8_t chd2 = this->reg(0x68).get();
  uint8_t chd3 = this->reg(0x02).get();

  if( chd1 == 0x41 ){
    ESP_LOGD(TAG, "TAS2780 chip found.");
    ESP_LOGD(TAG, "Reg 0x68: %d.", chd2 );
    ESP_LOGD(TAG, "Reg 0x02: %d.", chd3 );
  }
  else
  {
    ESP_LOGD(TAG, "TAS2780 chip not found.");
    this->mark_failed();
    return;
  }
  
  this->reg(TAS2780_PAGE_SELECT) = 0x00;
  this->reg(0x0e) = 0x44;
  this->reg(0x0f) = 0x40;
  this->reg(0x0a) = (2 << 4) | (3 << 2) | 2 ;

  this->reg(TAS2780_PAGE_SELECT) = 0x01;
  this->reg(0x17) = 0xc0;
  this->reg(0x19) = 0x00;
  this->reg(0x21) = 0x00;
  this->reg(0x35) = 0x74;

  this->reg(TAS2780_PAGE_SELECT) = 0xFD;
  this->reg(0x0d) = 0x0d;
  this->reg(0x3e) = 0x4a;
  this->reg(0x0d) = 0x00;


  this->reg(TAS2780_PAGE_SELECT) = 0x00;
  //Power Mode 2 (no external VBAT)
  this->reg(0x03) = 0xa8;
  this->reg(0x04) = 0xa1;
  this->reg(0x71) = 0x12; 
  

  //Set interrupt masks
  this->reg(TAS2780_PAGE_SELECT) = 0x00;
  //mask VBAT1S Under Voltage
  this->reg(0x3d) = 0xFF;
  //mask all PVDD and VBAT1S interrupts
  this->reg(0x40) = 0xFF;
  this->reg(0x41) = 0xFF;
  
  
  // set interrupt to trigger For 
  // 0h : On any unmasked live interrupts
  // 3h : 2 - 4 ms every 4 ms on any unmasked latched
  uint8_t reg_0x5c = this->reg(0x5c).get();
  this->reg(0x5c) = (reg_0x5c & ~0x03) | 0x00;   

  this->activate();
}


void TAS2780::loop() {
}



void TAS2780::dump_config(){

}

bool TAS2780::set_mute_off(){
  this->is_muted_ = false;
  return this->write_mute_();
}

bool TAS2780::set_mute_on(){
  this->is_muted_ = true;
  return this->write_mute_();
}

bool TAS2780::set_volume(float volume) {
  this->volume_ = clamp<float>(volume, 0.0, 1.0);
  return this->write_volume_();
}

bool TAS2780::is_muted() {
  return this->is_muted_;
}

float TAS2780::volume() {
  return this->volume_;
}

bool TAS2780::write_mute_() {
  if( this->is_muted_ ){
    this->reg(TAS2780_DVC) = 0xc9;  
  } else {
    this->write_volume_(); 
  }
  return true;
}

bool TAS2780::write_volume_() {
  /*
  V_{AMP} = INPUT + A_{DVC} + A_{AMP}
  
  V_{AMP} is the amplifier output voltage in dBV ()
  INPUT: digital input amplitude as a number of dB with respect to 0 dBFS
  A_{DVC}: is the digital volume control setting as a number of dB (default 0 dB)
  A_{AMP}: the amplifier output level setting as a number of dBV

  AMP_LEVEL[4:0] : @48ksps 11dBV - 21dBV [0x00, 0x14]
  */ 
  float attenuation = (1. - this->volume_) * 200.f + .5f;
  uint8_t dvc = clamp<uint8_t>(attenuation, 0, 0xc8);
  this->reg(TAS2780_DVC) = dvc; 
  
  uint8_t amp_level = 7; // 15dBV
  uint8_t reg_val = this->reg(TAS2780_CHNL_0).get();
  reg_val &= ~TAS2780_CHNL_0_AMP_LEVEL_MASK;
  reg_val |= amp_level << TAS2780_CHNL_0_AMP_LEVEL_SHIFT;
  this->reg(TAS2780_CHNL_0) = reg_val;

  return true;
}



}
}