@echo off
setlocal enabledelayedexpansion
::
:: Back up everything on the in-radio ATmega328 before overwriting it.
::
:: Reads flash, EEPROM, fuses and lock bits over ICSP with an FT232R in
:: synchronous bitbang mode. Flash is read TWICE and the dumps compared --
:: a marginal ISP connection gives readbacks that differ in a few bytes, and
:: a single read would look perfectly fine.
::
:: Wiring: see docs\ISP-WIRING.md. TXD->RESET, RXD->SCK, RTS->MISO, CTS->MOSI,
:: GND->GND. Do NOT connect VCC if the board is powered from the car.
::
:: Usage:  tools\backup-328.bat                        (FT232R bitbang)
::         tools\backup-328.bat arduino_as_isp COM5   (Arduino Mega as ISP)
::

set "AVRDUDE=C:\Users\latonita\scoop\apps\avrdude\current\avrdude.exe"
:: Programmer: ttl232r = FT232R bitbang (default), arduino_as_isp = Arduino/Mega
:: running the ArduinoISP sketch. The latter needs a COM port as arg 2.
set "PROG=%~1"
if "%PROG%"=="" set "PROG=ttl232r"
set "PORT=%~2"
set "PORTARG="
if not "%PORT%"=="" set "PORTARG=-P %PORT%"
:: ATmega328P. The KiCad symbol says ATmega328PB-AU, but the silicon reports
:: signature 1E 950F, which is a plain 328P -- trust the chip, not the schematic.
set "PART=m328p"

:: If reads are unreliable, slow the bit clock: set BITCLOCK=-B 8
set "BITCLOCK="

if not exist "%AVRDUDE%" (
  echo ERROR: avrdude not found at:
  echo   %AVRDUDE%
  echo.
  echo Install it with:  scoop install avrdude
  echo The Arduino IDE's avrdude will NOT work -- it has no FTDI support built in.
  exit /b 1
)

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd-HHmmss"') do set "TS=%%i"
set "OUT=%~dp0..\backup-328-%TS%"
mkdir "%OUT%" 2>nul
if not exist "%OUT%" (
  echo ERROR: could not create %OUT%
  exit /b 1
)

echo ==========================================================
echo  Backing up ATmega328 to:
echo    %OUT%
echo ==========================================================
echo.

echo [1/5] Checking the ISP connection...
"%AVRDUDE%" -c %PROG% -p %PART% %PORTARG% %BITCLOCK% >"%OUT%\signature.txt" 2>&1
if errorlevel 1 (
  echo.
  echo ERROR: the AVR is not responding. Nothing was read.
  echo.
  echo   * Check the wiring against docs\ISP-WIRING.md
  echo   * The FT232R must be set to 5V logic, not 3.3V
  echo   * The board needs power -- either VCC from the stick, or the car
  echo   * Close any serial monitor holding the port
  echo   * If it is intermittent, edit this file and set BITCLOCK=-B 8
  echo.
  type "%OUT%\signature.txt"
  exit /b 1
)
findstr /i "signature" "%OUT%\signature.txt"
echo       ...responding.
echo.

echo [2/5] Reading fuses and lock bits...
"%AVRDUDE%" -c %PROG% -p %PART% %PORTARG% %BITCLOCK% ^
  -U lfuse:r:"%OUT%\lfuse.hex":h ^
  -U hfuse:r:"%OUT%\hfuse.hex":h ^
  -U efuse:r:"%OUT%\efuse.hex":h ^
  -U lock:r:"%OUT%\lock.hex":h >>"%OUT%\avrdude.log" 2>&1
if errorlevel 1 (
  echo ERROR: fuse read failed. See %OUT%\avrdude.log
  exit /b 1
)
set /p LFUSE=<"%OUT%\lfuse.hex"
set /p HFUSE=<"%OUT%\hfuse.hex"
set /p EFUSE=<"%OUT%\efuse.hex"
set /p LOCK=<"%OUT%\lock.hex"
echo       lfuse=%LFUSE%  hfuse=%HFUSE%  efuse=%EFUSE%  lock=%LOCK%

:: 0xFF or 0x3F is "no locks" -- on a 328P the top two lock bits are unused and
:: read back as 1, so avrdude 8.x reports 0xFF for an unlocked part. Anything
:: else can block flash readback silently, returning 0xFF without an error.
echo %LOCK% | findstr /i "0x3f 0xff" >nul
if errorlevel 1 (
  echo.
  echo  ** WARNING: lock bits are %LOCK%, not the unlocked 0x3F.
  echo  ** Flash readback may be blocked and return all 0xFF WITHOUT an error.
  echo  ** Check flash.hex below before trusting this backup.
  echo.
)
echo.

echo [3/5] Reading flash ^(32 KB, first pass^)...
"%AVRDUDE%" -c %PROG% -p %PART% %PORTARG% %BITCLOCK% -U flash:r:"%OUT%\flash.hex":i >>"%OUT%\avrdude.log" 2>&1
if errorlevel 1 (
  echo ERROR: flash read failed. See %OUT%\avrdude.log
  exit /b 1
)

echo [4/5] Reading flash again to cross-check the connection...
"%AVRDUDE%" -c %PROG% -p %PART% %PORTARG% %BITCLOCK% -U flash:r:"%OUT%\flash-2nd.hex":i >>"%OUT%\avrdude.log" 2>&1
if errorlevel 1 (
  echo ERROR: second flash read failed. See %OUT%\avrdude.log
  exit /b 1
)
fc /b "%OUT%\flash.hex" "%OUT%\flash-2nd.hex" >nul
if errorlevel 1 (
  echo.
  echo  ** WARNING: the two flash dumps DIFFER.
  echo  ** The ISP connection is not reliable -- do not trust this backup.
  echo  ** Reseat the wiring, set BITCLOCK=-B 8 in this file, and run again.
  echo.
) else (
  echo       both passes identical -- readback is trustworthy.
  del "%OUT%\flash-2nd.hex"
)
echo.

echo [5/5] Reading EEPROM ^(1 KB^)...
"%AVRDUDE%" -c %PROG% -p %PART% %PORTARG% %BITCLOCK% -U eeprom:r:"%OUT%\eeprom.hex":i >>"%OUT%\avrdude.log" 2>&1
if errorlevel 1 (
  echo WARNING: EEPROM read failed -- continuing. See %OUT%\avrdude.log
)
echo.

:: A blank chip reads as nothing but 0xFF. Worth saying out loud, because it
:: means there is no firmware to recover and the backup is pointless.
findstr /v /i /c:":10" "%OUT%\flash.hex" >nul 2>&1
for %%A in ("%OUT%\flash.hex") do set "FLASHSZ=%%~zA"
echo   flash.hex is !FLASHSZ! bytes

> "%OUT%\RESTORE.txt" echo How to put this back on the ATmega328
>>"%OUT%\RESTORE.txt" echo =====================================
>>"%OUT%\RESTORE.txt" echo.
>>"%OUT%\RESTORE.txt" echo Captured: %TS%
>>"%OUT%\RESTORE.txt" echo Fuses read at backup time: lfuse=%LFUSE% hfuse=%HFUSE% efuse=%EFUSE% lock=%LOCK%
>>"%OUT%\RESTORE.txt" echo.
>>"%OUT%\RESTORE.txt" echo Restore flash and EEPROM:
>>"%OUT%\RESTORE.txt" echo.
>>"%OUT%\RESTORE.txt" echo   avrdude -c %PROG% -p %PART% %PORTARG% -U flash:w:flash.hex:i
>>"%OUT%\RESTORE.txt" echo   avrdude -c %PROG% -p %PART% %PORTARG% -U eeprom:w:eeprom.hex:i
>>"%OUT%\RESTORE.txt" echo.
>>"%OUT%\RESTORE.txt" echo Fuses: do NOT rewrite these unless something actually changed them.
>>"%OUT%\RESTORE.txt" echo Flashing does not touch fuses, so normally you can skip this. A wrong
>>"%OUT%\RESTORE.txt" echo LOW fuse can leave the chip needing an external clock injected before it
>>"%OUT%\RESTORE.txt" echo will talk at all.
>>"%OUT%\RESTORE.txt" echo.
>>"%OUT%\RESTORE.txt" echo   avrdude -c %PROG% -p %PART% %PORTARG% -U lfuse:w:%LFUSE%:m -U hfuse:w:%HFUSE%:m -U efuse:w:%EFUSE%:m
>>"%OUT%\RESTORE.txt" echo.
>>"%OUT%\RESTORE.txt" echo Wiring is in jeep-aux-and-can-gateway\docs\ISP-WIRING.md.

echo ==========================================================
echo  Done. Saved in:
echo    %OUT%
echo.
echo    flash.hex      the program, including any bootloader
echo    eeprom.hex     stored settings
echo    lfuse/hfuse/efuse/lock.hex
echo    RESTORE.txt    exact commands to put it back
echo    avrdude.log    full output if anything looked wrong
echo ==========================================================
endlocal
