// file created 26 April 2018

// Things to write/read EEPROM
struct config_t
{
  unsigned long wattHours;
  float wattHours_temp;
  float max_sol_watts;
} memory;

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}
// was in setup()
  // recover data from EEPROM
  /* EEPROM_readAnything(0, memory);
    if (memory.wattHours < wattHours_setup)
     wattHours = wattHours_setup;
    else
     wattHours = memory.wattHours;

    wattHours_temp = memory.wattHours_temp;
    max_sol_watts = memory.max_sol_watts;

  */

// was in others() function
    /*   memory.wattHours = wattHours;
       memory.wattHours_temp = wattHours_temp;
       memory.max_sol_watts = max_sol_watts;
       EEPROM_writeAnything(0, memory);
    */


