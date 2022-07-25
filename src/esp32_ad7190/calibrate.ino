

static int cmdState = 0;
static int cmdSubState = 0;
static int cmdErrorCount = 0;


/***************************************************************************//**
   @brief Calibrate AD7190.

   @param spiType - 1: HSPI, 2: VSPI.
   @param calType - 1: Zero, 2: Prop.

   @return none.
*******************************************************************************/
boolean calibrate_ad7190(uint8_t spiBus, AD7190_SPI * ad7190_spi, uint8_t calType)
{
  Serial.print("Do Calibrate, spiType=");
  Serial.print(spiBus);
  Serial.print(", calType=");
  Serial.println(calType);

  if (calType == 1) {
    Serial.println("Zero calibration");
    if (spiBus == HSPI) {
      if (ad7190_spi->isActive()) {
        hspiWeightZero = ad7190_spi->weightReadAvg(6);
        Serial.print("hspiWeightZero= ");
        Serial.println(hspiWeightZero);
      } else {
        Serial.println("Error: HSPI not active state");
        return false;
      }
    } else if (spiBus == VSPI) {
      if (ad7190_spi->isActive()) {
        vspiWeightZero = ad7190_spi->weightReadAvg(6);
        Serial.print("vspiWeightZero= ");
        Serial.println(vspiWeightZero);
      } else {
        Serial.println("Error: VSPI not active state");
        return false;
      }
    } else {
      Serial.print("Invalid spiBus");
      Serial.print(spiBus);
    }
    delay(1000);
  }
  else if (calType == 2) {
    Serial.println("100g calibration");
    if (spiBus == HSPI) {
      if (ad7190_spi->isActive()) {
        uint32_t weight_count = ad7190_spi->weightReadAvg(6);
        hspiWeightProportion = (weight_count - hspiWeightZero) * 1000 / 100;
        Serial.print("hspiWeightProportion= ");
        Serial.println( hspiWeightProportion);
      } else {
        Serial.println("Error: HSPI not active state");
        return false;
      }
    } else if (spiBus == VSPI) {
      if (ad7190_spi->isActive()) {
        uint32_t weight_count = ad7190_spi->weightReadAvg(6);
        vspiWeightProportion = (weight_count - vspiWeightZero) * 1000 / 100;
        Serial.print("vspiWeightProportion= ");
        Serial.println( vspiWeightProportion);
      } else {
        Serial.println("Error: VSPI not active state");
        return false;
      }
    } else {
      Serial.print("Invalid spiBus");
      Serial.print(spiBus);
    }
    delay(1000);
  } else {
    Serial.print("Invalid cal type:");
    Serial.println(calType);
    return false;
  }
  return true;
}



void process_cmd()
{

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'q') {
      Serial.println("Quit setup mode.");
      calibrateMode = false;
      cmdState = 0;
      cmdSubState = 0;
      cmdErrorCount = 0;
      return;
    }
    if (c == 0x0A) {
      return;
    }

    if (cmdErrorCount > 10) {
      Serial.print("Input error exceed 5 times, quit");
      cmdState = 0;
      cmdSubState = 0;
      cmdErrorCount = 0;
      calibrateMode = false;
      return;
    }

    switch (cmdState) {
      case 0:   // init state
        if (c == 's') {
          calibrateMode = true;
          Serial.println("In setup mode");
          Serial.println("Please select which SPI (1: HSPI; 2: VSPI; p: print value, q: Quit):");
          cmdState = 1;
          cmdErrorCount = 0;
        } else {
          ++cmdErrorCount;
          Serial.println("Input 's' to enter Setup mode");
        }
        break;
      case 1:  // SPI setup mode
        cmd_select_action(c);
        break;
      case 2: // HSPI config
        cmd_hspi_cal(c);
        break;
      case 3: // VSPI config
        cmd_vspi_cal(c);
        break;
      default:
        ++cmdErrorCount;
        cmdState = 0;
        cmdSubState = 0;
        Serial.println("Input command:");
        Serial.println("\ts: setup mode");
        Serial.println("\tq: quit");
    }
    //    Serial.print("cmdState=");
    //    Serial.print(cmdState);
    //    Serial.print(", cmdSubState=");
    //    Serial.println(cmdSubState);

  }
}

void process_cmd1()
{

  while (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'i':
        Serial.println("Set Instructment reinitializing...");
        ad7190_hspi_init(1);
        delay(1);
        break;
      case 'j':
        Serial.println("Set lite config...");
        ad7190_hspi_lite_config();
        delay(1);
        break;
    }
  }
}

void cmd_select_action(char c)
{
  if (c == '1') {
    if (hspiEnabled) {
      Serial.println("Config HSPI:");
      Serial.println("\tInput 'z' to calibrate zero");
      cmdState = 2;
      cmdSubState = 0;
    } else {
      Serial.println("Error: HSPI not Enabled");
      Serial.println("Please select which SPI (1: HSPI; 2: VSPI; q: Quit):");
    }
  }
  else if (c == '2') {
    if (vspiEnabled) {
      Serial.println("Config VSPI:");
      Serial.println("\tInput 'z' to calibrate zero");
      cmdState = 3;
      cmdSubState = 0;
    } else {
      Serial.println("Error: VSPI not Enabled");
      Serial.println("Please select which SPI (1: HSPI; 2: VSPI; q: Quit):");
    }
  } else if (c == 'p') {
    print_cal_value();
  } else {
    Serial.println("Please select which SPI (1: HSPI; 2: VSPI; p: print, q: Quit):");
  }
}

void cmd_hspi_cal(char c)
{
  switch (cmdSubState) {
    case 0:  // zero
      if (c == 'z') {
        Serial.println("HSPI Zero calibration");
        if (calibrate_ad7190(HSPI, ad7190_hspi, 1)) {
          Serial.println("HSPI Zero calbration finished.");
          Serial.println("Config VSPI:");
          Serial.println("\tInput 'c' to calibrate 100g");
          cmdSubState = 1;  // 100g
        } else {
          Serial.println("Calibration Error");
          cmdState = 1;
          cmdSubState = 0;
          Serial.println("Please select which SPI (1: HSPI; 2: VSPI; p: print, q: Quit):");
        }
      } else {
        Serial.print("Invalid command:");
        Serial.println(c);
        Serial.println("Input 'z' to calibrate zero");
        ++cmdErrorCount;
      }
      break;
    case 1: // 100g
      if (c == 'c') {
        Serial.println("HSPI 100g calibration");
        calibrate_ad7190(HSPI, ad7190_hspi, 2);
        Serial.println("HSPI Prop calibration finished.");
        cmdSubState = 0;  // 100g
        cmdState = 0; //VSPI
        Serial.println("Input 'q' to quit setup mode");
      } else {
        Serial.print("Invalid command:");
        Serial.println(c);
        Serial.println("Input 'c' to calibrate 100g");
        ++cmdErrorCount;
      }
  }
}

void cmd_vspi_cal(char c)
{
  switch (cmdSubState) {
    case 0:  // zero
      if (c == 'z') {
        Serial.println("VSPI Zero calibration");
        if (calibrate_ad7190(VSPI, ad7190_vspi, 1)) {
          Serial.println("HSPI Zero calbration finished.");
          Serial.println("Config VSPI:");
          Serial.println("\tInput 'c' to calibrate 100g");
          cmdSubState = 1;  // 100g
        } else {
          Serial.println("Calibration Error");
          cmdState = 1;
          cmdSubState = 0;
          Serial.println("Please select which SPI (1: HSPI; 2: VSPI; p: print, q: Quit):");
        }
      } else {
        Serial.print("Invalid command:");
        Serial.println(c);
        Serial.println("Input 'z' to calibrate zero");
        ++cmdErrorCount;
      }
      break;
    case 1: // 100g
      if (c == 'c') {
        Serial.println("VSPI 100g calibration");
        if (calibrate_ad7190(VSPI, ad7190_vspi, 2)) {
          Serial.println("VSPI Prop calibration finished.");
          cmdState = 0;
          cmdSubState = 0;
          Serial.println("Input 'q' to quit setup mode");
        } else {
          Serial.println("Calibration Error");
          cmdState = 1;
          cmdSubState = 0;
          Serial.println("Please select which SPI (1: HSPI; 2: VSPI; p: print, q: Quit):");
        }
      } else {
        Serial.print("Invalid command:");
        Serial.println(c);
        Serial.println("Input 'c' to calibrate 100g");
        ++cmdErrorCount;
      }
  }

}

void print_cal_value()
{
  Serial.println("-------------------------------------------------------------");
  Serial.print("HSPI: hspiWeightZero=");
  Serial.println(hspiWeightZero);
  Serial.print("HSPI: hspiWeightProportion=");
  Serial.println(hspiWeightProportion);
  Serial.println();
  Serial.print("VSPI: vspiWeightZero=");
  Serial.println(vspiWeightZero);
  Serial.print("VSPI: vspiWeightProportion=");
  Serial.println(vspiWeightProportion);
  Serial.println("-------------------------------------------------------------");

}
