/**
 * Emulates standard functions My Circuits 2022, based on code of David Bird 2018 
 * using command mode of OpenLOG. More info in video:
 *
 * https://www.youtube.com/watch?v=zoYMU1tA3nI
 *
 * Configuration file /config.txt on SD card is expected with contents:
 * 115200,26,1,2,1,1,0
 *
 * 115200 - Baudrate
 * 26 - Escape character to return to command mode
 * 1 - Escape character #
 * 2 - Mode; initially command mode (2)
 * 1 - Verbosity
 * 1 - Echo
 * 0 - Ignore RX 
 *
 * File name: esp32_up_down_sd.h
 * Date:      2024/04/06 19:18
 * Author:    ppet36
*/

#include <WiFi.h>
#include <ESP32WebServer.h>    //https://github.com/Pedroalbuquerque/ESP32WebServer download and place in your Libraries folder
#include <ESPmDNS.h>
#include "StringTokenizer.h"

#include "CSS.h" //Includes headers of the web and de style file

#include "esp32_up_down_sd.h"

#define OPENLOG_BAUD 115200
#define OPENLOG_RX 26
#define OPENLOG_TX 25
#define OPENLOG_RESET 27
#define OPENLOG_MAX_SIZE 32768
#define OPENLOG_TRANSFER_BUFF_SIZE (size_t)80

#define OPENLOG_ESC 26 // Ctrl+Z
#define OPENLOG_TIMEOUT 3000
#define OPENLOG_INPUT_PROMPT '>'
#define OPENLOG_OUTPUT_PROMPT '<'

// Webserver
ESP32WebServer server(80);

// OpenLOG connection
#define openLog Serial1

// Inidicates whether SD card is present
bool SD_present = false;

// Flag if append log file initiated.
bool append_active = false;

// Disk size in bytes
uint64_t disk_size = 0;

// Current fragment size
uint16_t current_log_fragment_size = 0;

// Current fragment index
uint16_t current_log_fragment_index = 1;

// Webpage creation space
String webpage;

// HH:MM:SS;speed;euc_temp;euc_dist_since_charge;euc_phase_current;bms_voltage;bms_current;battery_level;bms_temp;bat1_temp;bat2_temp;bms_balance_cur;
const char CSV_HEADERS[] PROGMEM = "Time;Speed;EUC temp;EUC dist. since charge;EUC phase current;BMS voltage;BMS current;Battery level;BMS temp;Battery temp1;Battery temp2;BMS ballance current;Latitude;Longitude;Altitude;Satellites;Hdop;";


// External logging function which prints to serial and display.
extern void logLine(String s);
extern void log(String s);
extern void logLine();
extern void displaySet(String field, String value);


// Display updates
void displaySetStatus (String status) {
  displaySet (F("InfoLabel"), status);
}

void displayClearStatus() {
  displaySet (F("InfoLabel"), "");
}

// Wait for character with OPENLOG_TIMEOUT timeout.
bool OL_wait_for_char (char ch, unsigned int offset = 0, char* payload = 0, size_t payload_size = 0) {
  unsigned long t = millis();
  off_t pos = 0;
  int c;
  int readed = 0;  

  memset(payload, 0, payload_size);

  Serial.print (F("Waiting for char \""));
  Serial.print (ch);
  Serial.print (F("\""));

  while (millis() - t < OPENLOG_TIMEOUT) {
    if (openLog.available()) {
      c = openLog.read();

      if (c == ch) {
        Serial.print (F("... Char found at position "));
        Serial.println (pos);
        return true;
      }

      readed++;

      if (payload && (pos < payload_size) && (readed > offset)) {
        payload[pos++] = c;
      }
    } else {
      yield();
    }
  }

  Serial.print (F("... Char not found in timeout "));
  Serial.println (OPENLOG_TIMEOUT);

  return false;
}

void OL_empty_queue() {
  while (openLog.available()) {
    openLog.read();
    delay(1);
  }
}

bool OL_send_command (String command, char waitFor = OPENLOG_INPUT_PROMPT, char* payload = 0, size_t payload_size = 0) {
  OL_empty_queue();

  openLog.print(command);
  openLog.write(13);
  Serial.print(F("Sent command \""));
  Serial.print(command);
  Serial.println(F("\""));
  if (OL_wait_for_char (waitFor, command.length(), payload, payload_size)) {
    return true;
  }
  return false;
}


void OL_reset(void) {
  digitalWrite (OPENLOG_RESET, LOW);
  delay(500);
  digitalWrite (OPENLOG_RESET, HIGH);
  delay (250);
  OL_send_command ("reset", OPENLOG_INPUT_PROMPT);
}

// Initialize connection to OpenLOG.
void SD_setup(void) {
  char diskInfo[512];

  logLine(F("Initializing OpenLOG..."));
  openLog.begin(OPENLOG_BAUD, SERIAL_8N1, OPENLOG_RX, OPENLOG_TX);

  if (OL_send_command ("disk", OPENLOG_INPUT_PROMPT, diskInfo, sizeof(diskInfo))) {
    Serial.print (diskInfo);

    char* needle = strstr (diskInfo, "Card Size:");
    if (needle) {
      if (sscanf (needle, "Card Size: %llu KB", &disk_size) == 1) {
        disk_size *= (1024 * 1024);
      }
    }

    log (F("Parsed disk size: "));
    log (String(disk_size));
    logLine (F(" bytes"));

    SD_present = (disk_size > 0);
  } else {
    SD_present = false;
    logLine (F("Failed to connect to OpenLOG..."));
  }

  log (F("SD_present is "));
  logLine(String(SD_present));
}

void SD_setup_wifi(void) {
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  
  //Set your preferred server name, if you use "mcserver" the address would be http://mcserver.local/
  if (!MDNS.begin(SERVERNAME)) {          
    logLine(F("Error setting up MDNS responder!"));
    delay (1000);
    ESP.restart(); 
  }
  
  /*********  Server Commands  **********/
  server.on("/",         SD_dir);
  server.on("/upload",   SD_file_upload);
  server.on("/fupload",  HTTP_POST,[](){ server.send(200);}, SD_handle_file_upload);

  server.begin();
  
  logLine("HTTP server started...");
  delay(500);
}

/*********  LOOP  **********/

void SD_loop_wifi(void) {
  server.handleClient();
}

bool save_current_log_fragment (DateTime time) {
  char file_name [20];

  sprintf (file_name, "/%04d%02d%02d.idx", time.year(), time.month(), time.day());
  if (OL_send_command("rm " + String(file_name)) && OL_send_command ("append " + String(file_name), OPENLOG_OUTPUT_PROMPT)) {
    openLog.println(current_log_fragment_index);

    Serial.print (F("Written current fragment index "));
    Serial.println (current_log_fragment_index);

    delay(500);
    return SD_command_mode();
  } else {
    return false;
  }
}

void SD_write_log (DateTime time, const char* value) {
  if (SD_present) {
    static uint8_t last_log_day = time.day();

    if (!append_active) {
      char file_name [20];
      char payload [32];
      bool new_file = false;

      sprintf (file_name, "/%04d%02d%02d.idx", time.year(), time.month(), time.day());
      if (OL_send_command ("size " + String(file_name), OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
        if (atoi(payload) < 0) {
          Serial.print (F("Fragment index file "));
          Serial.print (file_name);
          Serial.println (F(" not found."));

          current_log_fragment_index = 1;

          if (!save_current_log_fragment(time)) {
            SD_present = false;
            return;
          }
        } else {
          if (OL_send_command ("read " + String(file_name), OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
            current_log_fragment_index = atoi(payload);
            Serial.print (F("Current fragment index is "));
            Serial.println (current_log_fragment_index);
          } else {
            SD_present = false;
            return;
          }
        }
      } else {
        SD_present = false;
        return;
      }

      sprintf (file_name, "/%04d%02d%02d.%03d", time.year(), time.month(), time.day(), current_log_fragment_index);
      if (OL_send_command ("size " + String(file_name), OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
        int file_size = atoi (payload);
        Serial.print (F("File "));
        Serial.print (file_name);
        Serial.print (F(" size is "));
        Serial.print (file_size);
        Serial.println (F(" bytes."));

        new_file = (file_size < 0);
        current_log_fragment_size = new_file ? 0 : file_size;

      } else {
        SD_present = false;
        return;
      }

      if (!OL_send_command ("append " + String(file_name), OPENLOG_OUTPUT_PROMPT)) {
        SD_present = false;
        return;
      }

      if (new_file && (current_log_fragment_index < 2)) {
        openLog.print (CSV_HEADERS);
        openLog.println();
        current_log_fragment_size += strlen(CSV_HEADERS) + 2;
      }

      append_active = true;

    } else {
      if (last_log_day != time.day()) {
        Serial.print (F("Day changed "));
        Serial.print (last_log_day);
        Serial.print (F("<>"));
        Serial.print (time.day());
        Serial.println (F("; rotating."));

        if (SD_command_mode()) {
          append_active = false;
          last_log_day = time.day();
          SD_write_log (time, value);
          return;
        } else {
          SD_present = false;
          return;
        }
      }

      if (current_log_fragment_size > OPENLOG_MAX_SIZE) {
        Serial.print (F("Log max size "));
        Serial.print (OPENLOG_MAX_SIZE);
        Serial.println (F(" reached; rotating."));

        current_log_fragment_index++;

        if (SD_command_mode() && save_current_log_fragment (time)) {
          append_active = false;
          SD_write_log (time, value);
          return;
        } else {
          SD_present = false;
          return;
        }
      }
    }

    openLog.print(value);
    openLog.println();
    current_log_fragment_size += strlen(value) + 2;
  }
}

bool SD_command_mode(void) {
  unsigned long t = millis();
  do {
    Serial.println (F("Sending command mode character..."));
    openLog.write (OPENLOG_ESC);
    openLog.flush();
    if (OL_wait_for_char(OPENLOG_INPUT_PROMPT)) {
      Serial.println (F("Switched to command mode..."));
      return true;
    }
    delay (250);
  } while (millis() - t < OPENLOG_TIMEOUT * 3);

  Serial.println (F("Cant switch to command mode!"));

  return false;
}

void SD_flush(void) {
  if (SD_present) {
    if (append_active) {
      Serial.println (F("Append active; swiching to command mode..."));
      if (!SD_command_mode()) {
        return;
      }
      append_active = false;
    }

    if (!OL_send_command ("sync")) {
      SD_present = false;
      return;
    }
  }
}

bool SD_initialized() {
  return SD_present;
}

void SD_info(uint64_t* used_by_files, uint64_t* free, int* count_files) {
  unsigned long t = millis();
  char c;
  char line [128];
  char file_name [20];
  unsigned int file_size;
  off_t pos = 0;

  (* used_by_files) = 0;
  (* free) = 0;
  (* count_files) = 0;

  if (!SD_present) {
    return;
  }

  delay(500);
  SD_flush();

  memset(line, 0, sizeof(line));

  OL_empty_queue();

  openLog.print("ls");
  openLog.write(13);

  while (millis() - t < OPENLOG_TIMEOUT) {
    if (openLog.available()) {
      c = openLog.read();
      if (c == OPENLOG_INPUT_PROMPT) {
        break;
      }

      if (c == '\n') {
        if (sscanf (line, "%s %u", file_name, &file_size) == 2) {
          Serial.print (file_name);
          Serial.print ('\t');
          Serial.println (file_size);
          (* used_by_files) += file_size;
          (* count_files)++;
        }

        memset(line, 0, sizeof(line));
        pos = 0;
      }

      line[pos++] = c;
      if (pos >= sizeof(line)) {
        break;
      }
    }
  }

  (* free) = disk_size - (* used_by_files);
}

uint64_t SD_disk_size() {
  return disk_size;
}

/*********  FUNCTIONS  **********/
//Initial page of the server web, list directory and give you the chance of deleting and uploading
void SD_dir() {
  if (SD_present) {
    if (server.args() > 0) {
      String order = server.arg(0);
      Serial.print (F("Server arg[0] is "));
      Serial.println (order);

      if (order.indexOf("view_") >= 0) {
        order.remove(0,5);
        SD_file_download(order, "inline");
        return;
      }


      if (order.indexOf("download_") >= 0) {
        order.remove(0,9);
        SD_file_download(order, "attachment");
        return;
      }
  
      if (order.indexOf("delete_") >= 0) {
        order.remove(0,7);
        SD_file_delete(order);
        return;
      }
    }

    displaySetStatus (F("Reading SD files..."));

    SendHTML_Header();    
    webpage += F("<table align='center'>");
    webpage += F("<tr><th>Name</th><th>Size</th><th colspan='3'>Actions</th></tr>");
    SD_print_directory();
    webpage += F("</table>");
    SendHTML_Content();
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();

    displayClearStatus();
  } else {
    ReportSDNotPresent();
  }
}

//Upload a file to the SD
void SD_file_upload() {
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>"); 
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:25%' type='file' name='fupload' id = 'fupload' value=''>");
  webpage += F("<button class='buttons' style='width:10%' type='submit'>Upload File</button><br><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html",webpage);
}

//Prints the directory, it is called in void SD_dir() 
void SD_print_directory() {
  unsigned long t = millis();
  char c;
  char line [128];
  char file_name [20];
  char display_name [20];
  char payload [32];
  int file_size;
  off_t pos = 0;

  String lazyInitSizes = "";

  OL_reset();

  memset(line, 0, sizeof(line));

  OL_empty_queue();

  openLog.print("ls");
  openLog.write(13);

  while (millis() - t < OPENLOG_TIMEOUT * 2) {
    if (openLog.available()) {
      c = openLog.read();
      if (c == OPENLOG_INPUT_PROMPT) {
        break;
      }

      if (c == '\n') {
        if (sscanf (line, "%s %u", file_name, &file_size) == 2) {
          if (webpage.length() > 1000) {
            SendHTML_Content();
          }

          sprintf (display_name, "%s", file_name);
          String fsize = SD_file_size (file_size);

          int y = 0, m = 0, d = 0, part = 0;
          int scanned = sscanf (file_name, "%04d%02d%02d.%03d", &y, &m, &d, &part);

          if (scanned == 3) {
            sprintf (display_name, "%04d-%02d-%02d", y, m, d);
            fsize = "-";
            lazyInitSizes += String(file_name) + ",";
          }

          if (scanned != 4) {
            webpage += "<tr><td>" + String(display_name) + "</td>";
            webpage += "<td id='fs" + String(file_name) + "'>" + fsize + "</td>";
            webpage += "<td>";
            webpage += F("<FORM action='/' method='post'>"); 
            webpage += F("<button type='submit' name='view'");
            webpage += F(" value='"); webpage += "view_" + String(file_name); webpage += F("'>View</button>");
            webpage += "</td>";
            webpage += "<td>";
            webpage += F("<FORM action='/' method='post'>"); 
            webpage += F("<button type='submit' name='download'"); 
            webpage += F(" value='"); webpage += "download_" + String(file_name); webpage += F("'>Download</button>");
            webpage += "</td>";
            webpage += "<td>";
            webpage += F("<FORM action='/' method='post'>"); 
            webpage += F("<button type='submit' name='delete'");
            webpage += F(" value='"); webpage += "delete_" + String(file_name); webpage += F("'>Delete</button>");
            webpage += "</td>";
            webpage += "</tr>";
          }
        }   

        memset(line, 0, sizeof(line));
        pos = 0;
      }     

      line [pos++] = c;
      if (pos >= sizeof(line)) {
        break;
      }     
    }       
  }         

  webpage += "<script>";

  StringTokenizer sizes(lazyInitSizes, ",");
  while (sizes.hasNext()) {
    String fn = sizes.nextToken();
    String date_part = fn.substring(0, 8);
    uint32_t total_size = 0;

    for (int part = 1; part <= 999; part++) {
      char pn [32];
      sprintf (pn, "/%s.%03d", date_part.c_str(), part);

      if (OL_send_command ("size " + String(pn), OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
        file_size = atoi (payload);

        if (file_size < 0) {
          break;
        }

        total_size += file_size;
      }
    }

    webpage += "document.getElementById('fs" + fn + "').innerHTML = '" + SD_file_size (total_size) + "';";
  }
  webpage += "</script>";
}

off_t transfer_file_from_sd_card (String filename) {
  char buffer [OPENLOG_TRANSFER_BUFF_SIZE * 3 + 20];
  off_t pos = 0;
  char payload [32];

  displaySetStatus ("Transfering file " + filename + " ...");

  Serial.print (F("Starting file "));
  Serial.print (filename);
  Serial.println (F(" download..."));

  if (OL_send_command ("size " + filename, OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
    int file_size = atoi (payload);

    Serial.print (F("File "));
    Serial.print (filename);
    Serial.print (F(" size is "));
    Serial.print (file_size);
    Serial.println (F(" bytes."));

    while (pos < file_size) {
      size_t to_read = min (OPENLOG_TRANSFER_BUFF_SIZE, (size_t)(file_size - pos));
 
      // Read block of file in HEX
      String command = "read " + filename + " ";
      command += String(pos);
      command += " ";
      command += String(to_read);
      command += " 2";
 
      if (OL_send_command(command, OPENLOG_INPUT_PROMPT, buffer, sizeof(buffer))) { 
        char hexBuff [3];
        char decBuff [to_read];
        uint16_t decBuffIndex = 0;
        uint16_t bufferIndex = 0;
 
        memset(hexBuff, 0, sizeof(hexBuff));
 
        // Skip initial carriage returns
        while (bufferIndex < to_read * 3) {
          if (isprint (buffer[bufferIndex])) {
            break;
          }
          bufferIndex++;
        }
 
        while ((decBuffIndex < to_read) && (bufferIndex < sizeof(buffer))) {
          memcpy (hexBuff, &buffer[bufferIndex], 2);
 
          char ch = (char) strtol(hexBuff, NULL, 16);
          if ((ch == 0x0A) || (ch == 0x0D) ||  isprint (ch)) {
            decBuff[decBuffIndex] = ch;
            decBuffIndex++;
          } else {
            break;
          }
 
          // 0x0A, 0x0D comes as single character
          bufferIndex += (hexBuff[1] == ' ') ? 2 : 3;
        }
 
        Serial.println();
        Serial.print (F("Converted "));
        Serial.print (decBuffIndex);
        Serial.println (F(" bytes."));
 
        pos += decBuffIndex;
 
        server.client().write (decBuff, decBuffIndex);
 
        delay(250);
      } else {
        return -1;
      }
    }
 
    Serial.print(F("Transfered "));
    Serial.print(pos);
    Serial.println (F(" bytes."));
  } else {
    return -1;
  }

  displayClearStatus();

  return pos;
}


//Download a file from the SD, it is called in void SD_dir()
void SD_file_download (String filename, String content_disposition) {
  int file_size;
  size_t total_size = 0;
  char payload [256];

  OL_reset();

  server.sendHeader("Content-Type", "text/plain");
  server.sendHeader("Connection", "close");

  if (filename.endsWith(".idx")) {
    // Day metrics
    String date_part = filename.substring(0, 8);

    server.sendHeader ("Content-Disposition", content_disposition + "; filename=" + date_part + String(".csv"));

    uint64_t total_transfered = 0;
    uint16_t part_count = 0;

    for (int part = 1; part <= 999; part++) {
      char pn [32];
      sprintf (pn, "/%s.%03d", date_part.c_str(), part); 

      if (OL_send_command ("size " + String(pn), OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
        file_size = strtol (payload, NULL, 10);

        Serial.print (F("File "));
        Serial.print (pn);
        Serial.print (F(" size is "));
        Serial.print (file_size);
        Serial.println (F(" bytes."));

        if (file_size < 0) {
          break;
        }

        total_size += file_size;

        part_count++;
      }
    }

    Serial.print (F("Total files size "));
    Serial.print (total_size);
    Serial.println (F(" bytes."));

    server.setContentLength (total_size);
    server.send (200, "text/csv", "");

    delay(250);

    for (int part = 1; part <= part_count; part++) {
      char pn [32];
      off_t transfered;

      sprintf (pn, "%s.%03d", date_part.c_str(), part);

      transfered = transfer_file_from_sd_card (pn);
      if (transfered < 0) {
        Serial.print (F("Transfer of file "));
        Serial.print (pn);
        Serial.println (F(" failed."));
        break;
      }

      total_transfered += transfered;
    }

    Serial.print (F("Total transfered "));
    Serial.print (total_size);
    Serial.println (F(" bytes."));
  } else {
    // Plain file
    if (OL_send_command ("size " + filename, OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
      file_size = atoi (payload);

      Serial.print (F("File "));
      Serial.print (filename);
      Serial.print (F(" size is "));
      Serial.print (file_size);
      Serial.println (F(" bytes."));

      server.sendHeader ("Content-Disposition", content_disposition + "; filename=" + filename);
      server.setContentLength (file_size);
      server.send (200, "text/plain", "");

      transfer_file_from_sd_card (filename);
    }
  }
}

//Upload a new file to the Filing system
void SD_handle_file_upload() {

  HTTPUpload& uploadfile = server.upload(); //See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
                                            //For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if(uploadfile.status == UPLOAD_FILE_START) {
    String filename = uploadfile.filename;
    if(!filename.startsWith("/")) {
      filename = "/" + filename;
    }

    Serial.print(F("Starting upload of file "));
    Serial.println (filename);

    displaySetStatus ("Uploading file" + filename + "...");

    OL_send_command ("rm " + filename);

    OL_empty_queue();
    openLog.print("append ");
    openLog.print(filename);
    openLog.write(13);
    append_active = true;

    filename = String();
  } else if (uploadfile.status == UPLOAD_FILE_WRITE) {
    openLog.write (uploadfile.buf, uploadfile.currentSize);
    Serial.print (F("Written "));
    Serial.print (uploadfile.currentSize);
    Serial.println (F(" bytes."));
  } else if (uploadfile.status == UPLOAD_FILE_END) {
    SD_flush();

    Serial.print (F("File "));
    Serial.print (uploadfile.filename);
    Serial.println (F(" successfully uploaded..."));

    displayClearStatus();

    webpage = "";
    append_page_header();
    webpage += F("<h3>File was successfully uploaded</h3>"); 
    webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename+"</h2>";
    webpage += F("<h2>File Size: "); webpage += SD_file_size(uploadfile.totalSize) + "</h2><br><br>"; 
    webpage += F("<a href='/'>[Back]</a><br><br>");
    append_page_footer();

    server.send(200,"text/html",webpage);
  } else {
    ReportCouldNotCreateFile("upload");
  }
}

//Delete a file from the SD, it is called in void SD_dir()
void SD_file_delete(String filename) {
  if (SD_present) {
    displaySetStatus ("Deleting file " + filename + "...");
    
    SendHTML_Header();

    SD_flush();

    bool resl = true;
    if (filename.endsWith (".idx")) {
      String date_part = filename.substring(0, 8);
      char payload[32];

      if (OL_send_command ("read /" + filename, OPENLOG_INPUT_PROMPT, payload, sizeof(payload))) {
        int part_count = atoi (payload);

        Serial.print (F("Index file /"));
        Serial.print (filename);
        Serial.print (F(" has "));
        Serial.print (part_count);
        Serial.println (F(" part count."));

        for (int part = 1; part <= part_count; part++) {
          char pn [32];
          sprintf (pn, "/%s.%03d", date_part.c_str(), part); 

          if (!OL_send_command ("rm " + String(pn))) {
            resl = false;
          }
        }

        if (resl) {
          resl = OL_send_command ("rm " + filename);
        }
      } else {
        resl = false;
      }
    } else {
      resl = OL_send_command ("rm " + filename);
    }

    if (resl) {
      webpage += "<h3>File '"+filename+"' has been deleted</h3>"; 
      webpage += F("<a href='/'>[Back]</a><br><br>");
    } else { 
      webpage += F("<h3>File was not deleted - error</h3>");
      webpage += F("<a href='/'>[Back]</a><br><br>");
    }

    append_page_footer(); 
    SendHTML_Content();
    SendHTML_Stop();

    displayClearStatus();
  } else {
    ReportSDNotPresent();
  }
}

//SendHTML_Header
void SendHTML_Header() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate"); 
  server.sendHeader("Pragma", "no-cache"); 
  server.sendHeader("Expires", "-1"); 
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
  server.send(200, "text/html", ""); //Empty content inhibits Content-length header so we have to close the socket ourselves. 
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Content
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Stop
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); //Stop is needed because no content length was sent
}

//ReportSDNotPresent
void ReportSDNotPresent() {
  SendHTML_Header();
  webpage += F("<h3>No SD Card present</h3>"); 
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportCouldNotCreateFile
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//File size conversion
String SD_file_size(uint64_t bytes) {
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes) +" B";
  else if(bytes < (1024*1024))      fsize = String(bytes/1024.0,3) + " KB";
  else if(bytes < (1024*1024*1024)) fsize = String(bytes/1024.0/1024.0,3) + " MB";
  else                              fsize = String(bytes/1024.0/1024.0/1024.0,3) + " GB";
  return fsize;
}

void append_page_header() {
  webpage  = F("<!DOCTYPE html><html>");
  webpage += F("<head>");
  webpage += F("<title>EUC Metric Server</title>");
  webpage += F("<meta name='viewport' content='user-scalable=yes,initial-scale=1.0,width=device-width'>");
  webpage += F("<style>");
  webpage += F("body{max-width:65%;margin:0 auto;font-family:arial;font-size:100%;}");
  webpage += F("ul{list-style-type:none;padding:0;border-radius:0em;overflow:hidden;background-color:#d90707;font-size:1em;}");
  webpage += F("li{float:left;border-radius:0em;border-right:0em solid #bbb;}");
  webpage += F("li a{color:white; display: block;border-radius:0.375em;padding:0.44em 0.44em;text-decoration:none;font-size:100%}");
  webpage += F("li a:hover{background-color:#e86b6b;border-radius:0em;font-size:100%}");
  webpage += F("h1{color:white;border-radius:0em;font-size:1.5em;padding:0.2em 0.2em;background:#d90707;}");
  webpage += F("h2{color:blue;font-size:0.8em;}");
  webpage += F("h3{font-size:0.8em;}");
  webpage += F("table{font-family:arial,sans-serif;font-size:0.9em;border-collapse:collapse;width:85%;}"); 
  webpage += F("th,td {border:0.06em solid #dddddd;text-align:left;padding:0.3em;border-bottom:0.06em solid #dddddd;}"); 
  webpage += F("tr:nth-child(odd) {background-color:#eeeeee;}");
  webpage += F(".rcorners_n {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:20%;color:white;font-size:75%;}");
  webpage += F(".rcorners_m {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:50%;color:white;font-size:75%;}");
  webpage += F(".rcorners_w {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:70%;color:white;font-size:75%;}");
  webpage += F(".column{float:left;width:50%;height:45%;}");
  webpage += F(".row:after{content:'';display:table;clear:both;}");
  webpage += F("*{box-sizing:border-box;}");
  webpage += F("a{font-size:75%;}");
  webpage += F("p{font-size:75%;}");
  webpage += F("</style></head><body><h1>EUC Metric server</h1>");
  webpage += F("<ul>");
  webpage += F("<li><a href='/'>Days / Files</a></li>");
  webpage += F("<li><a href='/upload'>Upload</a></li>"); 
  webpage += F("</ul>");
}

//Saves repeating many lines of code for HTML page footers
void append_page_footer() { 
  webpage += F("</body></html>");
}

