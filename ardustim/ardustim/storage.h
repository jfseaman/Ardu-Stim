
#define EEPROM_CURRENT_VERSION  1

#define EEPROM_VERSION          1 //byte 1
#define EEPROM_CURRRENT_WHEEL   2 //byte 2 
#define EEPROM_CURRENT_RPM_MODE 3 //byte 3
#define EEPROM_CURRENT_RPM      4 //Note this is 2 bytes

void loadConfig();
void saveConfig();
