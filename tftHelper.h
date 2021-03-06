#ifndef tft_helper_h
#define tft_helper_h

// number of lines on tft
#define TFT_NB_OF_LINES 5

int tftCurrentLine = 0;
char tftBuffer[TFT_NB_OF_LINES+1][80]; // reserve one extra line for scrolling

void tftClear(TFT_eSPI* pTft) {
    pTft->setTextColor(TFT_GREEN, TFT_BLACK);
    pTft->fillScreen(TFT_BLACK);
    pTft->setTextDatum(TL_DATUM);
    tftCurrentLine=0;
 }

void tftPrintLine(TFT_eSPI* pTft, const char *buf, const int lineNb) {
  pTft->drawString(buf, 0, lineNb * 16);
}

void tftPrint(TFT_eSPI* pTft, const char *buf) {
  if (tftCurrentLine >= TFT_NB_OF_LINES) {
    // scroll & print
    tftClear(pTft);
    for (int ix=0; ix < TFT_NB_OF_LINES; ix++) {
      strncpy(tftBuffer[ix], tftBuffer[ix+1], sizeof(tftBuffer[ix]));
      tftPrintLine(pTft, tftBuffer[ix], ix);
    } 
  }
  else {
    // print
    tftPrintLine(pTft, tftBuffer[tftCurrentLine], tftCurrentLine);
    tftCurrentLine +=1;
  }
  
}

char* tfdGetBuffer() {
  return tftBuffer[tftCurrentLine];
}

#endif
