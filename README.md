Prerequisites: Install CC3200 SDK, import http_client_demo code.<br><br>
# http_client_demo
Settings for the serial port: 115200, 8N1, no handshake<br>
Enter your WiFi PASSWORD and SSID name in "common.h"<br>
Change location (strPtr = " & loc=Los Angeles \0"; // Your location.) to your geographical location in the source code main file, flush the project into your CC3200 board, and view the results on the www.cnktechlabs.com/data.html. <br>
Connect CC3200 to a serial terminal on your PC.<br>
Reset the board.<br>
<b>To enter password and SSID name either:</b><br>
Press and hold SW2 and reset the board<br>
On prompt "Enter Password" enter WiFi password<br>
On prompt "SSID name" enter WiFi name.<br>
<b>Or just reset the board and the common.h credentials will be used.</b>
