/**
 * Emulates standard functions My Circuits 2022, based on code of David Bird 2018 
 * using command mode of OpenLOG.
 *
 * File name: esp32_up_down_sd.h
 * Date:      2024/04/06 19:18
 * Author:    ppet36
*/
#ifndef __ESP32_UP_DOWN_SD_H__
#define __ESP32_UP_DOWN_SD_H__

#include <DS3231.h>

void SD_setup(void);
void SD_setup_wifi(void);
void SD_loop_wifi(void);
void SD_dir();
void SD_file_upload();
void SD_print_directory();
void SD_file_download(String filename, String content_disposition);
void SD_handle_file_upload();
void SD_file_delete(String filename);
void SendHTML_Header();
void SendHTML_Content();
void SendHTML_Stop();
void ReportSDNotPresent();
void ReportFileNotPresent(String target);
void ReportCouldNotCreateFile(String target);
String SD_file_size(uint64_t bytes);
void append_page_header();
void append_page_footer();
void SD_info(uint64_t* used_by_files, uint64_t* free, int* count_files);
uint64_t SD_disk_size();
bool SD_command_mode(void);
void SD_write_log (DateTime time, const char* value);
void SD_flush(void);
bool SD_initialized();

#endif

