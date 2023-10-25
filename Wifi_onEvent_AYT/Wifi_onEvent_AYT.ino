#include <WiFi.h>
#include <PubSubClient.h>
#include "ModbusRtu.h"
#include <ArduinoJson.h>

#define led_connection 41
#define led_published 42

/////////////////////////Set Up Network////////////////////////
const char* ssid = "MIC_2.4GHz";
const char* password = "999999999";

const char* mqtt_server = "192.168.100.136";

IPAddress local_IP(192, 168, 100, 206); // Static IP 
IPAddress gateway(192, 168, 100, 1);    // Gateway IP address
IPAddress subnet(255, 255, 255, 0);     // subnet
/////////////////////////Set Up Network////////////////////////

//////////////เพิ่มเติม////////////////////////////
int8_t state = 0;
char Machine_no[] = "TB01";
const int num = 14;   //จำนวน Register
uint16_t Data[num];
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
const unsigned long interval1 = 5000; //Send Data MQTT
const unsigned long interval2 = 2000;//Reconect MQTT
//////////////เพิ่มเติม////////////////////////////
Modbus slave(1, Serial1, 0);

WiFiClient espClient;
PubSubClient client(espClient);

esp_timer_handle_t timer_handle;
bool newData = false;
void onTimer(void* arg)
{
  state = slave.poll( Data , num );
  if(state >= 7){newData = true;}
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to WiFi");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      break;
    default:
      break;
  }
}

void reconnect()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    //Serial.println("Wifi connected");
    if (!client.connected())
    {
      if (client.connect("ESP32S2_Client")) {

      } else {
        Serial.println("MQTT Disconected");
        
        digitalWrite(led_published, LOW);
        //delay(5000);
      }
    } else {
      //Serial.println("MQTT connected");
    }
    client.loop();

  } else {
    Serial.println("Not connected");
    digitalWrite(led_published, LOW);
  }
}

void setup()
{
  Serial.begin(500000);
  Serial1.begin(115200);
  pinMode(led_connection, OUTPUT);
  pinMode(led_published, OUTPUT);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .name = "my_timer",
  };

  esp_timer_create(&timer_args, &timer_handle);

  esp_timer_start_periodic(timer_handle, 100);  // 100000 ไมโครวินาที = 100 มิลลิวินาที
  slave.start();
  client.setServer(mqtt_server, 1883); // MQTT broker address and port
}

void loop()
{
  digitalWrite(led_published, HIGH);
  digitalWrite(led_connection, HIGH);
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis1 >= interval1 && newData )
  {
    previousMillis1 = currentMillis;
    Serial.println("-----Reading ------- : ");
    int rssi = WiFi.RSSI();

    Serial.print("RSSI : "); Serial.println(rssi);
    Serial.print("1.D200 : "); Serial.println(Data[5]);
    Serial.print("2.D201 : "); Serial.println(Data[6]);
    Serial.print("3.D202 : "); Serial.println(Data[7]);
    Serial.print("4.D203 : "); Serial.println(Data[8]);
    Serial.print("5.D204 : "); Serial.println(Data[9]);
    Serial.print("6.D205 : "); Serial.println(Data[10]);
    Serial.print("7.D206 : "); Serial.println(Data[11]);
    Serial.print("8.D207 : "); Serial.println(Data[12]);
    Serial.print("9.D208 : "); Serial.println(Data[13]);
    //Serial.print("10.D209 : "); Serial.println(Data[14]);
    //Serial.print("11.D210 : "); Serial.println(Data[10]);
    digitalWrite(led_published, LOW);
    // สร้าง JSON object
    StaticJsonDocument<5000> doc;
    doc["rssi"] = rssi;
    doc["TB01/Run_Time"] = Data[5];  
    doc["TB01/Stop_Time"] = Data[6];
    doc["TB01/Daily_OK"] = Data[7];
    doc["TB01/Daily_NG"] = Data[8];
    doc["TB01/Daily_Tatal"] = Data[9];
    doc["TB01/Cycle_Time"] = Data[10];
    doc["TB01/Ball_C1_OK"] = Data[11];
    doc["TB01/Ball_C2_OK"] = Data[12];
    doc["TB01/Ball_C1_NG"] = Data[13];
    /*doc["TB01/Ball_C2_NG"] = Data[9];
    doc["TB01/D210"] = Data[10];
      doc["TB01/D211"] = d211;
      doc["TB01/D212"] = d212;
      doc["TB01/D213"] = d213;
      doc["TB01/D214"] = d214;
      doc["TB01/D215"] = d215;
      doc["TB01/D216"] = d216;
      doc["TB01/D217"] = d217;
      doc["TB01/D218"] = d218;
      doc["TB01/D219"] = d219;*/

    // แปลง JSON object เป็น string
    String jsonStr;
    serializeJson(doc, jsonStr);

    // ส่งข้อมูลผ่าน MQTT
    client.publish("TB01/json1", jsonStr.c_str());
    Serial.println(jsonStr);
    Serial.println("\n---------------finish loop------------------\n\n");
    newData = false;
  }
  
  if (currentMillis - previousMillis2 >= interval2 )
  {
    previousMillis2 = currentMillis;  // บันทึกเวลาปัจจุบัน
    reconnect() ;  
  }
}
