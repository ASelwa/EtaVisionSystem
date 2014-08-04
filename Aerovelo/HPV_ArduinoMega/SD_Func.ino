// SD functions
void sd_Init(){
  Serial.println("Initializing SD card...");
  Serial.print("_SS =");
  Serial.println(_SS);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(_SS, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(_SS)) {
    Serial.println("Card failed, or not present");
  }
  else{
    Serial.println("card initialized.");
  }
  return;

}

void sd_Write(char *data, char *filename){
//  if (START){
    //Write to SD card
    File dataFile = SD.open(filename, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(data);
      dataFile.close();
      //Serial.print("SD good. ");
      //Serial.println(filename);
    }  
    // if the file isn't open, pop up an error:
    else {
      //Serial.print("SD Write error. ");
      //Serial.println(filename);
    }
//  }
  return;
}

int32_t sd_ReadWord(File dataFile, char *_word){
	char temp;
	int n = 0;
	
    // read 
	temp = dataFile.read();
        
	while ((temp != ' ') && (temp != '/n')){
		_word[n] = temp;
                //Serial.print(_word[n]);
		temp = dataFile.read();
		++n;
                if (n > 16) break;
		}
		_word[n] = 0;
		return n;
}
/*
Saves data in the SD card
UNDONE
*/
void storeData(){
	sprintf(sdBuffer, "%li, %li, %li, %li, %li");
	return;
}
