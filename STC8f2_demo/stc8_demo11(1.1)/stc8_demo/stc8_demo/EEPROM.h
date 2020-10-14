#ifndef  __EEPROM__
#define  __EEPROM__


char IapRead(int addr);
void IapProgram(int addr, char dat);
void IapErase(int addr);

#endif
