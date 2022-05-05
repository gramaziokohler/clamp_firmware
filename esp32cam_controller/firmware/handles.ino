void handle_jpg_stream(void)
{
  Serial.println("Entered handle_jpg_stream()");
  ledcWrite(FLASH_LED_CHANNEL, FLASH_LED_BRIGHTNESS); // FLASH LED On
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (1)
  {
    cam.run();
    if (!client.connected()) {
      ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off
      break;
    }
    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);

    client.write((char *)cam.getfb(), cam.getSize());
    server.sendContent("\r\n");
    if (!client.connected()) {
      ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off
      break;
    }
  }
  Serial.println("Exit handle_jpg_stream()");
}

void handle_jpg(void)
{
  Serial.println("Entered handle_jpg()");
  ledcWrite(FLASH_LED_CHANNEL, FLASH_LED_BRIGHTNESS); // FLASH LED On
  delay(100);
  WiFiClient client = server.client();

  cam.run();
  if (!client.connected())
  {
    return;
  }
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-disposition: inline; filename=capture.jpg\r\n";
  response += "Content-type: image/jpeg\r\n\r\n";
  server.sendContent(response);
  client.write((char *)cam.getfb(), cam.getSize());
  ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off
}

void handleNotFound()
{
  Serial.println("Entered handleNotFound()");
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text/plain", message);
}
