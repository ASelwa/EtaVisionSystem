// SD functions
void sd_Init() {
  Serial.println("Initializing SD card...");
  Serial.print("_SS =");
  Serial.println(_SS);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(_SS, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(_SS)) {
    Serial.println("Card failed, or not present");
    // Display error message
    *((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
    *((uint8_t*)(slipBuffer + 1 + 0)) = 0;
    *((uint8_t*)slipBuffer + 1 + 1) = 0;
    SlipPacketSend(2, (char*)slipBuffer, &Serial3);
  }
  else {
    Serial.println("Card initialized.");
    // Display no OSD message if successful
    *((uint8_t*)slipBuffer + 0) = ID_SDCOMM;
    *((uint8_t*)(slipBuffer + 1 + 0)) = 1;
    *((uint8_t*)slipBuffer + 1 + 1) = 0;
    SlipPacketSend(2, (char*)slipBuffer, &Serial3);
  }
}

void sd_Write(char *data, char *filename) {
  // Write to SD card
  File dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(data);
    dataFile.close();
  }
}

void sd_Log(char *data) {
  //Write to SD card
  File dataFile = SD.open(logFilename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(data);
    dataFile.close();
  }
}

static File file;

/* Functions to write while manually opening and closing file */
////////////////////////////////////////////////////////////////
void sd_Open(char *filename) {
  file = SD.open(filename, FILE_WRITE);
}

void sd_Print(char *data) {
  if (file) {
    file.print(data);
  }
}

void sd_Close() {
  file.close();
}

////////////////////////////////////////////////////////////////

int32_t sd_ReadWord(File dataFile, char *_word) {
  char temp;
  int n = 0;

  // read
  temp = dataFile.read();

  while ((temp != ' ') && (temp != '/n')) {
    _word[n] = temp;
    temp = dataFile.read();
    ++n;
    if (n > 16) break;
  }
  _word[n] = 0;
  return n;
}
