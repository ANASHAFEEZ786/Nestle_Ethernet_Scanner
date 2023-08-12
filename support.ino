void ble_scan(void *pvParameter) {
  String chr_val;
  int int_d_val, collision_val_int;
  while (1) {
    BLEScanResults foundDevices = pBLEScan->start(1, false);
    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());
    Serial.println("Scan done!");
    // pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

        if(beaconInRange==2)
        {
          Serial.println("Beacon is VERY CLOSE!!!");
           digitalWrite(BUZ_PIN, HIGH);
          

          int err =0;
          Serial.println(dev_name);
          Serial.println(temp);
          String path = "/Nestle_Test/addMHE.php?mhe_id="+String(dev_name)+"&collision=1&collider="+String(temp);
//          /Nestle_Test/addMHE.php?mhe_id=ITECK_001&collision=1&collider=ITECK_001
          const char *kPath = path.c_str(); // Convert the String to a const char*
       
         
  EthernetClient c;
  HttpClient http(c);
//   http.begin(c,"https://iot.itecknologi.com/Nestle_Test/addMHE.php?mhe_id=ITECK_001&violation_time=2023-08-08 00:00:00&log_time=2023-08-08 00:00:00&collision=1&collider=ITECK_001");
  err = http.get(kHostname,kPath);
//  err = http.get;
  if (err == 0)
  {
    Serial.println("startedRequest ok");

    err = http.responseStatusCode();
    if (err >= 0)
    {
      Serial.print("Got status code: ");
      Serial.println(err);

      // Usually you'd check that the response code is 200 or a
      // similar "success" code (200-299) before carrying on,
      // but we'll print out whatever response we get

      err = http.skipResponseHeaders();
      if (err >= 0)
      {
        int bodyLen = http.contentLength();
        Serial.print("Content length is: ");
        Serial.println(bodyLen);
        Serial.println();
        Serial.println("Body returned follows:");
      
        // Now we've got to the body, so we can print it out
        unsigned long timeoutStart = millis();
        char c;
        // Whilst we haven't timed out & haven't reached the end of the body
        while ( (http.connected() || http.available()) &&
               ((millis() - timeoutStart) < kNetworkTimeout) )
        {
            if (http.available())
            {
                c = http.read();
                // Print out this character
                Serial.print(c);
               
                bodyLen--;
                // We read something, reset the timeout counter
                timeoutStart = millis();
            }
            else
            {
                // We haven't got any data, so let's pause to allow some to
                // arrive
                delay(kNetworkDelay);
            }
        }
      }
      else
      {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
      }
    }
    else
    {    
      Serial.print("Getting response failed: ");
      Serial.println(err);
    }
  }
  else
  {
    Serial.print("Connect failed: ");
    Serial.println(err);
  }
  http.stop();
         digitalWrite(BUZ_PIN, LOW);
       

//     http.begin("http://iot.itecknologi.com/automation/device.php?ip_address=192.168.2.1&device_module=AUTOMATION%20DEVICE%201&device_id=1");
//    int httpCode = http.GET(); 
//     if (httpCode > 0) { //Check the returning code
// 
//         //Get the request response payload
//      Serial.println(httpCode);             //Print the response payload
//     delay(1000);
//     }
//  while(1);
        }

    
//    switch (beaconInRange)
//  {
//  // case 0:
//  //   Serial.println("Beacon in Range");
//
//  //   for (int i = 0; i < 5; i++)
//  //   {
//  //     digitalWrite(BUZ_PIN, HIGH);
//  //     delay(400);
//  //     digitalWrite(BUZ_PIN, LOW);
//  //     delay(400);
//  //   }
//  //   break;
//  // case 1:
//  //   Serial.println("Beacon is CLOSE!");
//
//  //   for (int i = 0; i < 10; i++)
//  //   {
//  //     digitalWrite(BUZ_PIN, HIGH);
//  //     delay(200);
//  //     digitalWrite(BUZ_PIN, LOW);
//  //     delay(200);
//  //   }
//  //   break;
//  case 2:
//    Serial.println("Beacon is VERY CLOSE!!!");
//
//     http.begin("http://iot.itecknologi.com/automation/device.php?ip_address=192.168.2.1&device_module=AUTOMATION%20DEVICE%201&device_id=1");
//    int httpCode = http.GET(); 
//     if (httpCode > 0) { //Check the returning code
// 
//         //Get the request response payload
//      Serial.println(httpCode);             //Print the response payload
//     delay(1000);
//     }
////    for (int i = 0; i < 10; i++)
////    {
////      digitalWrite(BUZ_PIN, HIGH);
////      delay(50);
////      digitalWrite(BUZ_PIN, LOW);
////      delay(50);
////    }
//    break;
//  default:
//    Serial.println("Beacon not in Range");
//    break;
//  }

  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory
  beaconInRange = -1;
//    if (!strcmp(pCharacteristic->getValue().c_str(), "1")) {
//      myFile = SD.open("/operation_state.txt", FILE_WRITE);
//      myFile.println("1");
//      myFile.close();
//      Serial.println("Switching to WiFi Direct");
//      ESP.restart();
//    }
    chr_val = pCharacteristicD->getValue().c_str();
    int_d_val = chr_val.toInt();
    if (int_d_val != distance_val) {
      distance_val = int_d_val;
      EEPROM.write(0, distance_val);
      EEPROM.commit();

      // myFile = SD.open("/distance.txt", FILE_WRITE);
      // if (myFile) {
      //   myFile.println(String(distance_val));
      //   myFile.close();
      // }
    }
//    chr_val = pCharacteristicC->getValue().c_str();
//    collision_val_int = chr_val.toInt();
//    if (collision_val_int != collision_val) {
//      collision_val = collision_val_int;
//      EEPROM.write(32, collision_val);
//      EEPROM.commit();
      // myFile = SD.open("/collision.txt", FILE_WRITE);
      // if (myFile) {
      //   myFile.println(String(collision_val));
      //   myFile.close();
      // }
//    }
    // chr_val = pCharacteristicDN->getValue().c_str();
    // chr_val.trim();
    // dev_name.trim();
    // if (!chr_val.equals(dev_name)) {
    //   Serial.println("Device Name Changed");
    //     EEPROM.writeString(9,"ITECK_"+chr_val );
    //     EEPROM.commit();

    //   // myFile = SD.open("/dev_name.txt", FILE_WRITE);
    //   // if (myFile) {
    //     // myFile.println("ITECK_" + chr_val);
    //     // myFile.close();
    //     // ESP.restart();
    //   // }
    // }
//        chr_val = pCharacteristicDCC->getValue().c_str();
//        chr_val.trim();
//        col_cord_str.trim();
//         int collision_cor_int = chr_val.toInt();
//       if (collision_cor_int != collision_val_dur) {
//      collision_val_dur = collision_cor_int;
//      EEPROM.write(64, collision_val_dur);
//      EEPROM.commit();
//          determine_col_cord(col_cord_str);
        //   myFile = SD.open("/collision_cord.txt", FILE_WRITE);
        //   if(myFile) {
        //     myFile.println(chr_val);
        //  }
//        }
//    if (digitalRead(REG_PIN)) {
//      Serial.println("Driver Registration");
//      if (driver_reg) {
//        digitalWrite(LED_1, LOW);
//        driver_reg = false;
//      } else {
//        digitalWrite(LED_1, HIGH);
//        driver_reg = true;
//      }
//    }

//    if (!digitalRead(GPIO_pin)) {
//      Serial.println("Activity Detected");
//      collision = true;
//    }
//        if(analogRead(IGN_PIN) == 0)
//   {
//     ESP.restart();
//
//   } 

 

    // if (!(analogRead(IGN_PIN) > 0)) {
    //   myFile = SD.open("/operation_state.txt", FILE_WRITE);
    //   if (myFile) {
    //     myFile.println("2");
    //     myFile.close();
    //     ESP.restart();
    //   }
    // }
    vTaskDelay(10);
  }
}

//void collision_detect(void *pvParameter) {
//
//  while (1) {
//
//    if (mpu.getMotionInterruptStatus()) {
//      alert_type = 1;
//      Serial.println("collision registering");
//      myFile = SD.open("/report.txt", FILE_APPEND);
//      if (myFile) {
//        report_date.trim();
//        report_time.trim();
//        dev_name.trim();
//        DateTime rtc_time = rtc.now();
//        myFile.print("*" + rtc_time.timestamp(DateTime::TIMESTAMP_FULL) + "," + String(alert_type) + "," + dev_name + ",1,1,1,1,1#");
//        myFile.close();
//        Serial.println("*" + rtc_time.timestamp(DateTime::TIMESTAMP_FULL) + "," + String(alert_type) + "," + dev_name + ",1,1,1,1,1#");
//      }
//      collision = false;
//      buzzer_use = 1;
//      Serial.println("In Collision Buzzer");
//      for (int e = 0; e < 3; e++)
//      {
//        digitalWrite(BUZ_PIN, HIGH);
//        vTaskDelay(2000);
//        digitalWrite(BUZ_PIN, LOW);
//        vTaskDelay(500);
//        delay(500);
//      }
//      buzzer_use = 0;
//      Serial.println("Buzzer off");
//
//      collision = false;
//
////      pinMode(GPIO_pin, INPUT_PULLUP);
//      //      attachInterrupt(digitalPinToInterrupt(GPIO_pin), Ext_INT1_ISR, FALLING);
//    }
//    vTaskDelay(100);
//  }
//}
//bool rtc_check(DateTime rtc_time)
//{ 
//  if (rtc_time.year() > 2100 || rtc_time.year() == 2000 || rtc_time.day() > 31 || rtc_time.month() > 12 ||  rtc_time.month() == 0 || rtc_time.day() == 0)
//  {
//    return true;
//  }
//  else return false;
//}

void buzzer_alarm(void *pvParameter) {
  while (1) {
    if (!digitalRead(GPIO_pin)) {
      Serial.println("Activity Detected");
      collision = true;
    }
    if (play_buzzer) {
      int i = 3;
      while (i) {
        if (!digitalRead(GPIO_pin)) {
          Serial.println("Activity Detected");
          collision = true;
        }
        Serial.println("Buzzer Play");
        if (buzzer_use) {
          break;
        }
        digitalWrite(BUZ_PIN, 1);
        delay(100);
        if (buzzer_use) {
          break;
        }
        digitalWrite(BUZ_PIN, 0);
        delay(100);
        i--;
      }
      play_buzzer = 0;
    }
  }
}

//void wifi_function(void *pvParameter) {
//  while (1) {
//    server.handleClient();
//  }
//}
//
//void wifi_function_led(void *pvParameter) {
//  while (1) {
//    digitalWrite(LED_1, !digitalRead(LED_1));
//    vTaskDelay(500);
//  }
//}
//
//void send_response() {
//  String content;
//  content = "<!DOCTYPE HTML>\r\n<head><meta name='viewport' content='width=device-width,initial-scale=1,minimum-scale=1'></head><html><body>";
//  content += "<form method='get' action='datetime' onsubmit='return get_data_tim()'><input type='text' name='datetime' id='date_time' hidden> <input type='submit' value='Set Date and Time'></form><script>var currentdate = new Date();var datetime = currentdate.getDate()+'-'+(currentdate.getMonth()+1)+'-'+currentdate.getFullYear()+'-'+currentdate.getHours()+'-'+currentdate.getMinutes()+'-'+currentdate.getSeconds();document.getElementById('date_time').value = datetime;</script><br />";
//  content += "<form method='get' action='ssid'><input type='text' name='ssid' placeholder='Write SSID Name' />&nbsp;<input type='submit' value='Set SSID' /></form><br />";
//  if (ssid_str != "") {
//    content += "<spam style='color:green;font-weight:bold'><i>SSID set to <u>" + ssid_str + "</u> Successfully.</i></spam><br /><br />";
//  }
//  content += "<form method='get' action='password'><input type='text' name='password' placeholder='Write Password' />&nbsp;<input type='submit' value='Set Password' /></form><br />";
//  if (password_str != "") {
//    content += "<spam style='color:green;font-weight:bold'><i>Password set to <u>" + password_str + "</u> Successfully.</i></spam><br /><br />";
//  }
//  content += "<form method='get' action='server'><input type='text' name='server' placeholder='Write Server IP' />&nbsp;<input type='submit' value='Set Server IP' /></form><br />";
//  if (server_str != "") {
//    content += "<spam style='color:green;font-weight:bold'><i>Server IP set to <u>" + server_str + "</u> Successfully.</i></spam><br /><br />";
//  }
//  content += "<form method='get' action='port'><input type='text' name='port' placeholder='Write Server Port' />&nbsp;<input type='submit' value='Set Server Port' /></form><br />";
//  if (port_str != "") {
//    content += "<spam style='color:green;font-weight:bold'><i>Server Port set to <u>" + port_str + "</u> Successfully.</i></spam><br /><br />";
//  }
//  content += "<form method='get' action='switch'><input type='submit' value='Switch to Scan Mode'></form>";
//  if (datetime_show) {
//    content += "<script>window.alert('Date and time saved successfully');</script>";
//    datetime_show = 0;
//  }
//  if (file_downlaod) {
//    content += "<script>window.alert('File Not Found');</script>";
//    file_downlaod = 0;
//  }
//  content += "</body></html>";
//  ssid_str = ""; password_str = ""; server_str = ""; port_str = "";
//  server.send(200, "text/html", content);
//}
//
//void set_ssid() {
//  ssid_str = server.arg("ssid");
//  myFile = SD.open("/ssid.txt", FILE_WRITE);
//  myFile.println(ssid_str + "\n");
//  myFile.close();
//  ssid = (char*)ssid_str.c_str();
//  send_response();
//}
//
//void set_password() {
//  password_str = server.arg("password");
//  myFile = SD.open("/password.txt", FILE_WRITE);
//  myFile.println(password_str + "\n");
//  myFile.close();
//  password = (char*)password_str.c_str();
//  send_response();
//}

//void set_server_ip() {
//  server_str = server.arg("server");
//  myFile = SD.open("/server.txt", FILE_WRITE);
//  myFile.println(server_str + "\n");
//  myFile.close();
//  hostserver = (char*)server_str.c_str();
//  //hostserver = server_str.c_str();
//  send_response();
//}

//void set_server_port() {
//  port_str = server.arg("port");
//  myFile = SD.open("/port.txt", FILE_WRITE);
//  myFile.println(port_str + "\n");
//  myFile.close();
//  str_to_uint16(port_str.c_str(), &port);
//  send_response();
//}

//void switch_to_ble() {
//  myFile = SD.open("/operation_state.txt", FILE_WRITE);
//  myFile.println("0");
//  myFile.close();
//  ESP.restart();
//}

//void direct_wifi_mode() {
//  IPAddress local_IP(192, 168, 0, 2);
//  IPAddress gateway(192, 168, 0, 1);
//  IPAddress subnet(255, 255, 255, 0);
//  WiFi.softAPConfig(local_IP, gateway, subnet);
//  WiFi.softAP("iTeckDevice", "iteck12345");
//  IPAddress myIP = WiFi.softAPIP();
//  Serial.print("AP IP address: ");
//  Serial.println(myIP);
//  server.on("/", send_response);
//  server.on("/download", SD_file_download);
//  server.on("/datetime", set_date_time);
//  server.on("/switch", switch_to_ble);
//  server.on("/ssid", set_ssid);
//  server.on("/password", set_password);
//  server.on("/server", set_server_ip);
//  server.on("/port", set_server_port);
//  server.begin();
//  Serial.println("HTTP server started");
//}

//void SD_file_download() {
//  Serial.println(server.arg("download"));
//  String file_name = "/" + server.arg("download") + ".txt";
//  File download = SD.open(file_name);
//  if (download) {
//    server.sendHeader("Content-Type", "text/text");
//    server.sendHeader("Content-Disposition", "attachment; filename=" + file_name);
//    server.sendHeader("Connection", "close");
//    server.streamFile(download, "application/octet-stream");
//    download.close();
//  } else {
//    file_downlaod = 1;
//    send_response();
//  }
//}

//String getValue(String data, char separator, int index)
//{
//  int found = 0;
//  int strIndex[] = { 0, -1 };
//  int maxIndex = data.length() - 1;
//
//  for (int i = 0; i <= maxIndex && found <= index; i++) {
//    if (data.charAt(i) == separator || i == maxIndex) {
//      found++;
//      strIndex[0] = strIndex[1] + 1;
//      strIndex[1] = (i == maxIndex) ? i + 1 : i;
//    }
//  }
//  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
//}

//void set_date_time() {
//  datetime_show = 1;
//  String datetime = server.arg("datetime");
//  String ddate = getValue(datetime, '-', 0);
//  String dmonth = getValue(datetime, '-', 1);
//  String dyear = getValue(datetime, '-', 2);
//  String dhour = getValue(datetime, '-', 3);
//  String dmin = getValue(datetime, '-', 4);
//  String dsec = getValue(datetime, '-', 5);
//  rtc.adjust(DateTime(dyear.toInt(), dmonth.toInt(), ddate.toInt(), dhour.toInt(), dmin.toInt(), dsec.toInt()));
//  DateTime rtc_time = rtc.now();
//  Serial.println(rtc_time.timestamp(DateTime::TIMESTAMP_FULL));
//  Serial.println("Date and time saved successfully");
//  send_response();
//}

//void determine_col_cord(String cord_temp_str) {
//  char sz[col_cord_str.length()];
//  cord_temp_str.toCharArray(sz, cord_temp_str.length());
//  char *p = sz;
//  char *strtemp;
//  col_x_bool = false;
//  col_y_bool = false;
//  col_z_bool = false;
//  while ((strtemp = strtok_r(p, ",", &p)) != NULL) {
//    if(*strtemp == 'x') {
//      col_x_bool = true;
//    }
//    if(*strtemp == 'y') {
//      col_y_bool = true;
//    }
//    if(*strtemp == 'z') {
//      col_z_bool = true;
//    }
//  }
//}

//void wifi_tcp_connection(void *pvParameter) {
//  bool send_data_to_server = false;
//  while (1) {
//    Serial.println("Moving");
//    myFile = SD.open("/report.txt", FILE_READ);
//    long int myFile_size = myFile.size();
//    if (myFile_size > 5) {
//      send_data_to_server = true;
//    } else {
//      send_data_to_server = false;
//    }
//    myFile.close();
//    if (send_data_to_server) {
//      Serial.println("Waiting for data");
//      digitalWrite(LED_4, HIGH);
//      dev_name.trim();
//      while (tcp_reply != "SEND NOW") {
//        String data_to_send = "D=" + dev_name + "&FS=" + String(myFile_size - 1);
//        Serial.println(data_to_send);
//        client.print(data_to_send);
//      if (analogRead(IGN_PIN) > 0) {
//          // myFile = SD.open("/operation_state.txt", FILE_WRITE);
//          // if (myFile) {
//          //   myFile.println("0");
//          //   myFile.close();
//          operation_state=0;
//            digitalWrite(LED_4, LOW);
//            ESP.restart();
//          // }
//        }
//        vTaskDelay(2000);
//      }
//      myFile = SD.open("/report.txt", FILE_APPEND);
//      if (myFile) {
//        myFile.print("\n");
//        myFile.close();
//      }
//      while (tcp_reply != "OK") {
//        myFile = SD.open("/report.txt", FILE_READ);
//        Serial.print("Sending Data");
//        if (myFile) {
//          char character;
//          String datax = "";
//          while (character != '\n') {
//            char character = myFile.read();
//            if (character != '\n') {
//              datax += character;
//            } else {
//              break;
//            }
//            if (character == '#') {
//              client.print(datax);
//              datax = "";
//            }
//          }
//          myFile.close();
//        }
//        if (analogRead(IGN_PIN) > 0) {
//          // myFile = SD.open("/operation_state.txt", FILE_WRITE);
//          // if (myFile) {
//          //   myFile.println("0");
//          //   myFile.close();
//          operation_state=0;
//            digitalWrite(LED_4, LOW);
//            ESP.restart();
//          // }
//        }
//        delay(15000);
//      }
//      Serial.println("Data Sent Successfull");
//      client.stop();
//      myFile = SD.open("/report.txt", FILE_WRITE);
//      myFile.print("");
//      myFile.close();
//      digitalWrite(LED_4, LOW);
//    } else {
//      myFile.close();
//      client.stop();
//      Serial.println("moving into while");
//      while (!(analogRead(IGN_PIN) > 0));
//      // myFile = SD.open("/operation_state.txt", FILE_WRITE);
//      // if (myFile) {
//      //   myFile.println("0");
//      //   myFile.close();
//      operation_state=0;
//        digitalWrite(LED_4, LOW);
//        ESP.restart();
//      
//    }
//  }
//}
//
//void wifi_tcp_connection_read(void *pvParameter) {
//  tcp_reply = "";
//  while (1) {
//    while (client.connected()) {
//      while (client.available() > 0) {
//        String req = client.readStringUntil('\n');
//        req.trim();
//        tcp_reply = "";
//        tcp_reply = req;
//        Serial.println("TCP Reply: " + tcp_reply);
//        Serial.println("Data Received");
//      }
//    }
//    vTaskDelay(10);
//  }
//}
//
//bool str_to_uint16(const char *tempstr, uint16_t *res) {
//  char *end;
//  errno = 0;
//  long val = strtol(tempstr, &end, 10);
//  if (errno || end == tempstr || *end != '\0' || val < 0 || val >= 0x10000) {
//    return false;
//  }
//  *res = (uint16_t)val;
//  return true;
//}
//
//void sdcardcheck()
//{
//  ssidread = false, passread = false, serverread = false, portread = false, nameread = false, driverread = false, operationread = false, distanceread = false, collisionread = false, collisionsecondread = false;
//  myFile = SD.open("/ssid.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        ssidread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (ssidread == true)
//  {
//    myFile = SD.open("/ssid.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    ssidread = false;
//  }
//
//  myFile = SD.open("/password.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        passread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (passread == true)
//  {
//    myFile = SD.open("/password.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    passread = false;
//  }
//
//  myFile = SD.open("/server.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        serverread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (serverread == true)
//  {
//    myFile = SD.open("/server.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    serverread = false;
//  }
//
//  myFile = SD.open("/port.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        portread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (portread == true)
//  {
//    myFile = SD.open("/port.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    portread = false;
//  }
//
//  myFile = SD.open("/dev_name.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        nameread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (nameread == true)
//  {
//    myFile = SD.open("/dev_name.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    nameread = false;
//  }
//
//  myFile = SD.open("/driver.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        driverread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (driverread == true)
//  {
//    myFile = SD.open("/driver.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    driverread = false;
//  }
//
//  myFile = SD.open("/operation_state.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        operationread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (operationread == true)
//  {
//    myFile = SD.open("/operation_state.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    operationread = false;
//  }
//
//  myFile = SD.open("/distance.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        distanceread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (distanceread == true)
//  {
//    myFile = SD.open("/distance.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    distanceread = false;
//  }
//
//  myFile = SD.open("/collision.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        collisionread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (collisionread == true)
//  {
//    myFile = SD.open("/collision.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    collisionread = false;
//  }
//
//  myFile = SD.open("/collision_cord.txt", FILE_READ);
//  if (myFile) {
//    char character;
//    while (myFile.read() != '\n') {
//      //character = myFile.read();
//      if (myFile.read() == -1)
//      {
//        collisionsecondread = true;
//        break;
//      }
//    }
//    myFile.close();
//  }
//  if (collisionsecondread == true)
//  {
//    myFile = SD.open("/collision_cord.txt", FILE_APPEND);
//    myFile.println("");
//    myFile.close();
//    collisionsecondread = false;
//  }
//}
//void mpu6(void *pvParameter) {
//  while(1)
//  {
//     if(mpu.getMotionInterruptStatus()) {
//      digitalWrite(2,HIGH);
//    delay(1000);
//      Serial.println("Collision");
//      }
//      digitalWrite(2,LOW);
//  delay(5);
//     
//  }
//  vTaskDelay(100);
//}
