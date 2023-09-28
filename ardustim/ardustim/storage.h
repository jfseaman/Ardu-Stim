/* Atmega328p total EEPROM space is 1k 
   Space available for storing wheel definition is 1024 - size of settings
   At the moment that is 10 so space for wheel is 1014
   
   A scheme for storing the wheel definitions without destroying backward
   compatability. That will begin version 2. Storage of wheel def in EEPROM.
   ---
   As of 2023/09/21: The largest definition included in PROGMEM is 768 bytes. 
   Some of these are actually in SRAM as regular "literals" in the wheel
   structure. SRAM space used is:
   (sizeof(float)+sizeof(uint16_t+sizeof(uint16_t)) *  58 wheels = 464 bytes
   Since structures allow for abstraction of the order/location
   of member elements, I am going to reorder the structure to make it easier
   to store in EEPROM. The new structure members order is:
   _wheels.rpm_scaler       // float 4 bytes, in non-AVR archictures it may need to be cast to native size
   _wheels.wheel_degrees    // uint16_t 2 bytes,
   _wheels.wheel_max_edges  // uint16_t 2 bytes.
   _wheels.edge_states[]    // This is the beginning of wheel edge states bytes the lengthe of the array is .wheel_max_edges long
   _wheels.decoder_name[]   // This is the friendly name of the wheel zero terminated string. It begins at structure beginning + sizeof(rpm_scaler)+sizeof(wheel_degrees)+sizeof(wheel_max_edges)+wheel_max_edges+1 
*/
#define EEPROM_CURRENT_VERSION  1

#define EEPROM_VERSION          1 //byte 1
#define EEPROM_CURRRENT_WHEEL   2 //byte 2 
#define EEPROM_CURRENT_RPM_MODE 3 //byte 3
#define EEPROM_CURRENT_RPM      4 //Note: this is 2 bytes 4 AND 5
#define EEPROM_CURRENT_MAX      6 //Note: this is 2 bytes 6 and 7
#define EEPROM_CURRENT_IDLE     8 //Note: this is 2 bytes 8 and 9
#define EEPROM_CURRENT_CRANK    10//byte 10, flag for cranking in POT mode or not

void loadConfig();
void saveConfig();
void SetRpmShift();
