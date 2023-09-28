#include "storage.h"
#include "EEPROM.h"
#include "wheel_defs.h"
#include "user_defaults.h"
#include "ardustim.h"
#include "enums.h"

void loadConfig()
{
  switch(EEPROM.read(EEPROM_VERSION)) {
    case 255:   //New arduino signature "erased" EEPROM is set to FF
      selected_wheel = THIRTY_SIX_MINUS_ONE; //36-1
      wanted_rpm = 3000;
      mode = POT_RPM;
      set_rpm_idle = 0; // No idle speed set
      break;
      
    case 1:     // Fist version of ardustim EEPROM, contains no wheel definition
      selected_wheel = EEPROM.read(EEPROM_CURRRENT_WHEEL);
      
      mode = EEPROM.read(EEPROM_CURRENT_RPM_MODE);
  
      byte highByte = EEPROM.read(EEPROM_CURRENT_RPM);
      byte lowByte =  EEPROM.read(EEPROM_CURRENT_RPM+1);
      wanted_rpm = word(highByte, lowByte);
      
      highByte = EEPROM.read(EEPROM_CURRENT_MAX);
      lowByte =  EEPROM.read(EEPROM_CURRENT_MAX+1);
      set_rpm_cap = word(highByte, lowByte);
  
      // Set the idle speed. Error checking is 
      highByte = EEPROM.read(EEPROM_CURRENT_IDLE);
      lowByte =  EEPROM.read(EEPROM_CURRENT_IDLE+1);
      set_rpm_idle = word(highByte, lowByte);
  
      // Set if we are cranking on start up
      set_rpm_crank = EEPROM.read(EEPROM_CURRENT_CRANK);
      
      //Error checking
      if (set_rpm_cap > TMP_RPM_CAP) set_rpm_cap = TMP_RPM_CAP;
      if(selected_wheel >= MAX_WHEELS) { selected_wheel = 5; }
      if(mode >= MAX_MODES) { mode = FIXED_RPM; }
      if(wanted_rpm > TMP_RPM_CAP) wanted_rpm = 3000; // The rpm stored in EEPROM was unreasonable set it to middle ground.
      if (set_rpm_idle > set_rpm_cap) set_rpm_idle = 0;   // Sanity check. If it was uninitialized EEPROM spacd it would be 0xFFFF
  
      // Set up the bit shift based on max rpm (set_rpm_cap)
      SetRpmShift();
      break;
      
    case 2:     // Second version of ardustom EEPROM structure, wheel definitions saved
      break;
    default:
      break;
  }
}

void saveConfig()
{
  EEPROM.update(EEPROM_CURRRENT_WHEEL, selected_wheel);
  EEPROM.update(EEPROM_CURRENT_RPM_MODE, mode);
  EEPROM.update(EEPROM_VERSION, EEPROM_CURRENT_VERSION);

  byte highByte = highByte(wanted_rpm);
  byte lowByte = lowByte(wanted_rpm);
  EEPROM.update(EEPROM_CURRENT_RPM, highByte);
  EEPROM.update(EEPROM_CURRENT_RPM+1, lowByte);
  
  highByte = highByte(set_rpm_cap);
  lowByte = lowByte(set_rpm_cap);
  EEPROM.update(EEPROM_CURRENT_MAX, highByte);
  EEPROM.update(EEPROM_CURRENT_MAX+1, lowByte);
  
  highByte = highByte(set_rpm_idle);
  lowByte = lowByte(set_rpm_idle);
  EEPROM.update(EEPROM_CURRENT_IDLE, highByte);
  EEPROM.update(EEPROM_CURRENT_IDLE+1, lowByte);

  EEPROM.update(EEPROM_CURRENT_CRANK, set_rpm_crank);
}
