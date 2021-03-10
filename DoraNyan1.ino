/* DoraNyan1.ino
 * DoraNyan robot program for M5stack
 * The MIT License (MIT)
 * Copyright (c) 2020 haespo All rights reserved.
 * 
 * M5Stack library - MIT License
 * Copyright (c) 2017 M5Stack
 * https://opensource.org/licenses/mit-license.php
 * 引用：https://www.mgo-tec.com/blog-entry-m5stack-websocket-message-board-esp32.html/2
 * 引用：http://blog-yama.a-quest.com/?eid=970188
 * 
 */


#include <M5Stack.h>
#include <Wire.h>
#include "ESP32_LCD_ILI9341_SPI.h" //beta ver 1.2-
#include "ESP32_Button_Switch.h"
#include "ESP32_SD_ShinonomeFNT.h" //beta ver 1.22-
#include "ESP32_SD_UTF8toSJIS.h" //beta ver 1.22-
#include "ESP32_SD_EasyWebSocket.h" //beta ver 1.60-
#include "driver/i2s.h"
#include "aquestalk.h"

// GPIO settings
#define PIN_M5SPK     25
#define PIN_M5SPKMUTE 13
#define PIN_TOUCH     2  // T0:gpio4,T1:0,T2:2,T3:15,T4:13,T5:12,T6:14,T7:27,T8:33,T9:32
#define PIN_SERVO     5  // same of LEDC_GPIOPIN GPIO #36～#39 は設定不可
// AquesTalk
#define LEN_FRAME 32
uint32_t workbuf[AQ_SIZE_WORKBUF];
char strbuff[64];
// touch sensor
#define TS_SOUND      ("nya'-o")
#define TS_INITCNTNUM 16      // the number of initial meas data for desision threshold
#define TS_INITAVGNUM 4       // the number of data to average for desision threshold
#define TS_THSRATIO   (0.93)  // threshold ratio of the touch sensitivity
boolean f_touchdet = false;   // flug true:detected(touched), false:no det(untouched)
int touch_ths = 0;            // touch threshold level
// Moving Tail by servo of pwm driving by ledc
#define LEDC_CH       3         //channel max 15, Noise occurs from SP when using ch0
#define LEDC_BASEFREQ 50.0      // in hz
#define LEDC_TIMBIT   16        //max 16bit, 8bitの場合、最大周波数 312500Hz
#define LEDC_GPIOPIN  (PIN_SERVO) //GPIO #36～#39 は設定不可
#define SERVO_MIN_WIDTH_MS  0.6 // in ms
#define SERVO_MAX_WIDTH_MS  2.4 // in ms
#define TAILPOS_ON    89
#define TAILPOS_OFF   (-89)
#define TAILMOVETIME  2000      // tail moving time
boolean f_tail = false;         // tail position
// TIME
unsigned long time_tailmove;  // count time
// Wi-FI
const int8_t sck = 18; // SPI clock pin
const int8_t miso = -1; // MISO(master input slave output) don't using
const int8_t mosi = 23; // MOSI(master output slave input) pin
const int8_t cs = 14; // Chip Select pin
const int8_t dc = 27; // Data/Command pin
const int8_t rst = 33; // Reset pin
const int8_t LCD_LEDpin = 32;
 
const uint8_t CS_SD = 4; //SD card CS ( Chip Select )
 
const char* ssid = "aterm-26d859-g"; //ご自分のルーターのSSIDに書き換えてください
const char* password = "2da9d307cc425"; //ご自分のルーターのパスワードに書き換えてください
 
const char* UTF8SJIS_file = "/font/Utf8Sjis.tbl"; //UTF8 Shift_JIS 変換テーブルファイル名を記載しておく
const char* Shino_Zen_Font_file = "/font/shnmk16.bdf"; //全角フォントファイル名を定義
const char* Shino_Half_Font_file = "/font/shnm8x16.bdf"; //半角フォントファイル名を定義
 
const char* HTM_head_file1 = "/EWS/LIP2hed1.txt"; //HTMLヘッダファイル1
const char* HTM_head_file2 = "/EWS/LIP2hed2.txt"; //HTMLヘッダファイル2
const char* dummy_file = "/EWS/dummy.txt"; //HTMLファイル連結のためのダミーファイル
 
ESP32_LCD_ILI9341_SPI LCD(sck, miso, mosi, cs, dc, rst, LCD_LEDpin);
ESP32_SD_ShinonomeFNT SFR(CS_SD, 40000000); //SPI 24MHz
ESP32_Button_Switch BTN;
SD_EasyWebSocket ews;
 
IPAddress LIP; //ローカルIPアドレス自動取得用
 
//------LCD文字表示系　引数初期化------------
uint8_t max_txt = 40; //1x1倍 最大文字表示数
uint8_t ws_txt_sj_txt[ 400 ] = {}; //Shift_JIS文字コード格納
uint16_t ws_txt_sj_length;
uint8_t ws_txt_font_buf[2][16] = {}; //16x16フォント全角１文字格納バッファ
uint8_t Scl_Buf[ 16 ][ 640 ] = {}; //文字列スクロールpixelバッファ
//65k color red (0-31), green (0-63), blue (0-31)
uint8_t red = 31, green = 63, blue = 31;
uint8_t V_size = 3, H_size = 3; //V_size(垂直方向文字サイズ)、H_size(水平方向サイズ)
uint8_t prev_V_size = V_size;
uint16_t X0 = 0, Y0 = 0;
int32_t scl_speed = 0; //スクロール速度（0が最速)
bool scl_pause = false;
int8_t Scl_Cnt = {}; //文字スクロールカウント
uint16_t Fnt_Cnt = {}; //フォント半角１文字スクロールカウント
uint8_t Zen_or_Han = {}; //フォント読み取り時に関数から返ってきた全角または半角かの数値を格納
uint8_t num = 0; //文字列番号

//-----ボタンスイッチ　引数初期化-----------
const uint8_t buttonA_GPIO = 39;
const uint8_t buttonB_GPIO = 38;
const uint8_t buttonC_GPIO = 37;
uint8_t btn_stateA = _Release;
uint8_t btn_stateB = _Release;
uint8_t btn_stateC = _Release;
boolean V_size_down = false;
 
//------Easy WebSpclet関連　引数初期化----------------
String ret_str; //ブラウザから送られてくる文字列格納用
int PingSendTime = 10000; //ESP32からブラウザへPing送信する間隔(ms)
bool get_http_req_status = false; //ブラウザからGETリクエストがあったかどうかの判定変数
uint8_t WS_Status = 0;

//-----発話pronounce引数初期化-----------
boolean f_pronOnOff  = false;  // flug true:ON(発声), false:off(発声停止)
String  pron_str; // pronounce string

//***********セットアップ************************************************************************************
void setup() {
  // initialize the M5Stack object
  M5.begin();
  M5.Power.begin();
  // port setup
  pinMode(PIN_M5SPK, OUTPUT);   // M5stack speaker
  pinMode(PIN_M5SPKMUTE, OUTPUT);   // M5stack speaker Mute port
  pinMode(buttonA_GPIO, INPUT); //GPIO #39 は内部プルアップ無し
  pinMode(buttonB_GPIO, INPUT); //GPIO #38 は内部プルアップ無し
  pinMode(buttonC_GPIO, INPUT); //GPIO #37 は内部プルアップ無し
  ledcSetup(LEDC_CH, LEDC_BASEFREQ, LEDC_TIMBIT); // pwm config for servo
  ledcAttachPin(LEDC_GPIOPIN, LEDC_CH);           // pwm pin config for servo
  // serial
  Serial.begin(115200);
  // init touch (decision the touch threshold by filtered touch datas)
  int t[TS_INITCNTNUM];
  for(int i = 0; i < TS_INITCNTNUM; i++){   // read touch value TS_INITCNTNUM times
    t[i] = touchRead(PIN_TOUCH);
    if(i >= (TS_INITCNTNUM - TS_INITAVGNUM)) touch_ths += t[i]; // only some new data is read
    delay(100);
  }
  touch_ths = (touch_ths * TS_THSRATIO) / TS_INITAVGNUM ; // decision threshold
  printa("touch threshold check");          // std out
  for(int i = 0; i < TS_INITCNTNUM; i++){
    M5.Lcd.printf("%d ",t[i]);
    Serial.print(t[i]);Serial.print(", ");
  }
  M5.Lcd.printf("\r\ntouch threshold = %d\r\n",touch_ths);
  Serial.print("touch threshold = ");Serial.println(touch_ths);
  // init AquesTalk
  int iret;
  printa("Initializing AquesTalk...");
  iret = CAqTkPicoF_Init(workbuf, LEN_FRAME, "XXX-XXX-XXX");
  if(iret){
    printa("ERR:CAqTkPicoF_Init");
  }
  printa("Done!\r\n");
  // init servo
  servo_on(TAILPOS_ON);
  delay(2000);
  servo_on(TAILPOS_OFF);
  // font setup
  SFR.SD_Shinonome_Init3F(UTF8SJIS_file, Shino_Half_Font_file, Shino_Zen_Font_file); //ライブラリ初期化。3ファイル同時に開く
  LCD.ILI9341_Init(false, 40000000); //microSDを使う場合、必ず false にする
  LCD.Display_Clear();
  LCD.Brightness(0); //LCD LED Full brightness
 
  //---------オープニング画面表示--------------
  String test_str[ 4 ] ;
  test_str[0] = "Ｍ５ｓｔａｃｋ";
  test_str[1] = "Wi-Fi WebSocket";
  test_str[2] = "Message Board";
  test_str[3] = "by microSD";
 
  uint8_t test_buf[ 4 ][ 17 ][ 16 ] = {};
  uint16_t test_sj_len[ 4 ] = {};
 
  for( num = 0; num < 4; num++ ){
    test_sj_len[ num ] = SFR.StrDirect_ShinoFNT_readALL(test_str[ num ], test_buf[ num ]);
    Serial.printf("test_sj_len = %d\r\n", test_sj_len[ num ]);
  }
  num = 0, X0 = 48;
  LCD.HVsizeUp_8x16_Font_DisplayOut(2, 3, test_sj_len[ num ], X0, num * 48, red, green, blue, test_buf[ num ]);
  num = 1, X0 = 48;
  LCD.HVsizeUp_8x16_Font_DisplayOut(2, 3, test_sj_len[ num ], X0, num * 48, red, green, blue, test_buf[ num ]);
  num = 2, X0 = 48;
  LCD.HVsizeUp_8x16_Font_DisplayOut(2, 3, test_sj_len[ num ], X0, num * 48, red, green, blue, test_buf[ num ]);
  num = 3, X0 = 80;
  LCD.HVsizeUp_8x16_Font_DisplayOut(2, 3, test_sj_len[ num ], X0, num * 48, red, green, blue, test_buf[ num ]);
  X0 = 0;
  for( int i = 0; i < 256; i++){
    LCD.Brightness(i);
    delay(10);
  }
 
  //--------メッセージウィンドウ表示---------
  red = 15, green = 30, blue = 15;
  Status_Message("WiFi Connecting...", red, green, blue);
 
  //--------Wi-Fiアクセスポイント接続---------
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println(F("WiFi connected"));
  LIP = WiFi.localIP(); //ESP32のローカルIPアドレスを自動取得
  Serial.println( LIP );
  red = 31, green = 63, blue = 31;
  Status_Message( "IP : " + LIP.toString(), red, green, blue);
 
  ews.EWS_server_begin(); //ESP32 server生成
  LCD.Display_Clear(0, 0, 319, 4 * 48 - 1);
 
  //---------スクロール文字列のセットアップ-------------
  num = 0, red = 31, green = 63, blue = 31;
  LCD.Scrolle_Font_SetUp(num, max_txt, red, green, blue);
  Serial.printf("Free Heap Size = %d\r\n", esp_get_free_heap_size());
 
  //---------マルチタスク定義-------------------
  TaskHandle_t th; //マルチタスクハンドル定義
  xTaskCreatePinnedToCore(Task1, "Task1", 8192, NULL, 5, &th, 0); //マルチタスク起動
  
}

//***********メインループ************************************************************************************
void loop() {
  WebSocket_handshake(); //WebSocket ハンドシェイクでブラウザにHTML送信
  WebSocket_txt_receive(); //ブラウザからのテキスト受信
  check_touch();  // タッチされたかチェック

  if( !scl_pause ){ //文字列スクロール
    num = 0, X0 = 0, Y0 = 0;
    if( LCD.Scrolle_Inc_HVsizeUp_8x16_Font_DisplayOut(num, Zen_or_Han, scl_speed, H_size, V_size, &Scl_Cnt, ws_txt_sj_length, X0, Y0, ws_txt_font_buf, Scl_Buf) ){
      Zen_or_Han = SFR.Sjis_inc_FntRead(ws_txt_sj_txt, ws_txt_sj_length, &Fnt_Cnt, ws_txt_font_buf);
    }
 
    if( V_size_down ) {
      //ボタン操作の Task1 で、文字サイズダウンした場合、
      //LCDはVSPI接続でmicroSDと共用のため、同じループタスクで画面消去を行う。
      LCD.Display_Clear( 0, V_size * 16 + Y0 - 1, 319, prev_V_size * 16 + Y0 - 1 );
      V_size_down = false;
    }
    prev_V_size = V_size;
  }
 
  if( ews.WebSocket_Status() != WS_Status ){
    WS_Status = ews.WebSocket_Status();
    switch( WS_Status ){
      case 0:
        red = 31, green = 0, blue = 0;
        Status_Message( "WebSocket CLOSE", red, green, blue);
        break;
      case 1:
        red = 0, green = 63, blue = 31;
        Status_Message( "WebSocket Connected", red, green, blue);
        break;
    }
    Serial.printf("WS_Status = %d\r\n", WS_Status);
    Serial.printf("ews.WebSocket_Status() = %d\r\n", ews.WebSocket_Status());
  }

  // talking
  if(f_pronOnOff == true){
    playw(pron_str.c_str());
    f_pronOnOff = false;
  }
  // moving tail on
  else if(f_touchdet == true){
    playw(TS_SOUND);
    f_touchdet = false;
    servo_on(TAILPOS_ON);
    f_tail = true;
    time_tailmove = millis();
  }
  // moving tail off
  if(f_tail == true){
    unsigned long time_now = millis();
    if(time_now - time_tailmove > TAILMOVETIME){
      servo_on(TAILPOS_OFF);
      f_tail = false;
    }
  }

}

//************************************************************************************
/* servo PWM control
*/
static void servo_on(int degree){
  if(degree <= 90 && degree >= -90){
    float dd = (degree + 90) / 180.0 ;
    ledcWrite(LEDC_CH, (int)(65536 * (SERVO_MIN_WIDTH_MS + dd * (SERVO_MAX_WIDTH_MS -SERVO_MIN_WIDTH_MS)) / 20.0));
  }
}
static void servo_off(void){
  ledcWrite(LEDC_CH, 0x00);
}

/* Print All wrapper
*/
static void printa(const char* str){
  M5.Lcd.printf("%s\r\n",str);
  Serial.println(str);
}

/* Play wrapper
 *  dac create and release
*/
static void playw(const char* str){
  printa(str);
  DAC_Create();
  spk_on();
  Play(str);
  spk_off();
  DAC_Release();
}

/* M5stack speaker amp on/off (enable/disable)
   アンプのEnable端子をgpioでコントロール*/
static void spk_off(void){
  digitalWrite(PIN_M5SPKMUTE, HIGH);
}
static void spk_on(void){
  digitalWrite(PIN_M5SPKMUTE, LOW);
}

// touch sensor func. call main roop frequently
// touch_val0 : previous data for noise removal
void check_touch(void){
  static int touch_val0;
  int touch_val1 = touchRead(PIN_TOUCH);
  Serial.print("Touch = ");Serial.println(touch_val1);
  if((touch_val1 < touch_ths) && (touch_val0 < touch_ths)){ // comparison val and ths
    f_touchdet = true;
  }
  touch_val0 = touch_val1;
}


//************ マルチタスクループ ****************************
void Task1(void *pvParameters) {
  while(1){
    button_action();
    delay(1); //most important
  }
}
//******************************************************
void WebSocket_txt_receive(){
  ret_str = ews.EWS_ESP32CharReceive(PingSendTime);
  if(ret_str != "_close"){
    if(ret_str != "\0"){
      Serial.println(ret_str);
      if(ret_str != "Ping"){
        if(ret_str[0] == 't'){
          //ブラウザからのテキスト文字列受信
          String txt = ret_str.substring(ret_str.indexOf('|')+1, ret_str.length()-1);
          Serial.println( txt );
          StrFontConv( txt );
        }
        else if(ret_str[0] == 'p'){
          //発音pronounceデータ取得
          String txt = ret_str.substring(ret_str.indexOf('|')+1, ret_str.length()-1);
          Serial.print("pronounce = ");Serial.println( txt );
          pron_str = txt;       // save to pronounce resistor
          f_pronOnOff = true;   // speak start
        }
        else {
          int ws_data = (ret_str[0]-0x30)*100 + (ret_str[1]-0x30)*10 + (ret_str[2]-0x30);
          uint8_t picker_red;
          uint8_t picker_green;
          uint8_t picker_blue;
          float deg;
          float deg2;
          char col_c_65k[9] = {}; //ILI9341 65kカラー文字列収納
          char col_c_code[8] = {}; //HTMLカラーコード文字列格納
          String col_str;
 
          switch(ret_str[4]){
            case 'M': //スクロールスタート
              scl_pause = false;
              red = 0, green = 63, blue = 0;
              Status_Message( "Scrolle Start", red, green, blue);
              break;
            case 'N': //スクロール一時停止
              scl_pause = true;
              red = 31, green = 0, blue = 0;
              Status_Message( "Scrolle Pause", red, green, blue);
              break;
            case 'S': //スクロール速度、文字サイズアップ
              switch(ret_str[9]){
                case '1': //スクロール速度設定
                  scl_speed = 100 - floor(ws_data/2);
                  break;
                case '2': //水平文字サイズ変更
                  H_size = floor(ws_data/20);
                  if( H_size > 10 ) H_size = 10;
                  if( H_size < 1 ) H_size = 1;
                  Serial.printf("H_size = %d\r\n", H_size);
                  break;
                case '3': //垂直文字サイズ変更
                  V_size = floor(ws_data/20);
                  if( V_size > 10 ) V_size = 10;
                  if( V_size < 1 ) V_size = 1;
                  if( prev_V_size > V_size ) V_size_down = true;
                  Serial.printf("V_size = %d\r\n", V_size);
                  break;
              }
              break;
            case 'C': //カラーピッカー数値変換
              //HTMLカラーコード変換
              picker_red = (ret_str[10]-0x30)*100 + (ret_str[11]-0x30)*10 + (ret_str[12]-0x30);
              picker_green = (ret_str[14]-0x30)*100 + (ret_str[15]-0x30)*10 + (ret_str[16]-0x30);
              picker_blue = (ret_str[18]-0x30)*100 + (ret_str[19]-0x30)*10 + (ret_str[20]-0x30);
              sprintf( col_c_code, "#%02X%02X%02X", picker_red, picker_green, picker_blue );
 
              //ILI9341 65k カラー変換
              deg = (float)31 / (float)255 ;
              deg2 = (float)63 / (float)255 ;
              red = round( (float)picker_red * deg );
              green = round( (float)picker_green * deg2 );
              blue = round( (float)picker_blue * deg );
              Serial.printf("red, green, blue = %d, %d, %d\r\n", red, green, blue );
              LCD.Scrolle_Font_SetUp(0, max_txt, red, green, blue);
              sprintf( col_c_65k, "%02d,%02d,%02d", red, green, blue );
              col_str = String( col_c_code ) + "  ";
              col_str += String( col_c_65k );
              Status_Message( col_str, red, green, blue);
              break;
          }
        }
        ret_str = "";
      }
    }
  }else if(ret_str == "_close"){
    Serial.println("---------------WebSocket Close");
    ret_str = "";
  }
}
//******** Status メッセージ表示***************
void Status_Message(String str, uint8_t Red, uint8_t Green, uint8_t Blue){
  //ILI9341 ディスプレイに 半角19文字まで表示可能
  uint8_t f_buf[ 40 ][ 16 ] = {};
  uint16_t len = SFR.StrDirect_ShinoFNT_readALL(str, f_buf);
  if( len > 38 ) len = 19;
  X0 = 8, Y0 = ( 4 * 48 + 1 ) + 10;
  LCD.Display_Clear(1, Y0 + 1, 318, 238);
  LCD.HVsizeUp_8x16_Font_DisplayOut(2, 2, len, X0, Y0, Red, Green, Blue, f_buf);
  LCD.Draw_Rectangle_Line(0, 4 * 48, 319, 239, 31, 63, 31);
}
//********* String 文字列フォント変換***********
void StrFontConv(String str){
  Fnt_Cnt = 0;
  ws_txt_sj_length = SFR.UTF8toSJIS_convert(str, ws_txt_sj_txt);
  Zen_or_Han = SFR.Sjis_inc_FntRead(ws_txt_sj_txt, ws_txt_sj_length, &Fnt_Cnt, ws_txt_font_buf);
  Serial.printf("ws_txt_sj_length = %d\r\n", ws_txt_sj_length);
  Scl_Cnt = 0;
}
//********** ボタンスイッチ操作関数**************
void button_action(){
  btn_stateA = BTN.Button(0, buttonA_GPIO, true, 10, 300);
  switch( btn_stateA ){
    case _MomentPress:
      Serial.println("Button A Moment Press");
      V_size--;
      if(V_size < 1) V_size = 1;
      if( prev_V_size > V_size ) V_size_down = true;
      Serial.printf("V_size =%d\r\n", V_size);
      break;
    case _ContPress:
      Serial.println("-------------Button A Cont Press");
      V_size++;
      if(V_size > 10) V_size = 10;
      Serial.printf("V_size =%d\r\n", V_size);
      break;
    default:
      break;
  }
 
  btn_stateB = BTN.Button(1, buttonB_GPIO, true, 10, 300);
  switch( btn_stateB ){
    case _MomentPress:
      Serial.println("Button B Moment Press");
      H_size--;
      if(H_size < 1) H_size = 1;
      Serial.printf("H_size =%d\r\n", H_size);
      break;
    case _ContPress:
      Serial.println("-------------Button B Cont Press");
      H_size++;
      if(H_size > 15) H_size = 15;
      Serial.printf("H_size =%d\r\n", H_size);
      break;
    default:
      break;
  }
 
  btn_stateC = BTN.Button(2, buttonC_GPIO, true, 10, 300);
  switch( btn_stateC ){
    case _MomentPress:
      Serial.println("Button C Moment Press");
      scl_speed++;
      if(scl_speed > 100) scl_speed = 100;
      Serial.printf("scl_speed =%d\r\n", scl_speed);
      break;
    case _ContPress:
      Serial.println("-------------Button C Cont Press");
      scl_speed--;
      if(scl_speed < 0) scl_speed = 0;
      Serial.printf("scl_speed =%d\r\n", scl_speed);
      break;
    default:
      break;
  }
}
//***********************************************
void WebSocket_handshake(){  
  if(ews.Get_Http_Req_Status()){ //ブラウザからGETリクエストがあったかどうかの判定
    red = 15, green = 30, blue = 15;
    Status_Message( "WS connecting", red, green, blue );
 
    String html_str1="", html_str2="", html_str3="", html_str4="", html_str5="", html_str6="", html_str7="";
    //※String変数一つにEWS_Canvas_Slider_T関数は２つまでしか入らない
    html_str1 += "<body style='background:#000; color:#fff;'>\r\n";
    html_str1 += "<font size=3>\r\n";
    html_str1 += "ESP-WROOM-32(ESP32)\r\n";
    html_str1 += "<br>\r\n";
    html_str1 += "SD_EasyWebSocket Beta1.60 Sample\r\n";
    html_str1 += "</font><br>\r\n";
    html_str1 += ews.EWS_BrowserSendRate();
    html_str1 += "<br>\r\n";
    html_str1 += ews.EWS_Status_Text2("WebSocket Status","#555", 20,"#FF00FF");
    html_str1 += "<br><br>\r\n";
    html_str2 += "Set Messages  \r\n";
    html_str2 += ews.EWS_TextBox_Send("txt1", "おは今晩ちわ  ","送信");
    html_str2 += "<br><br>\r\n";
    html_str2 += "Set Pronunciation  \r\n";
    html_str2 += ews.EWS_TextBox_Send("pronounce", "ohakonbanchiwa","送信");
    html_str2 += "<br><br>\r\n";
    html_str2 += "Scrolle  \r\n";
    html_str2 += ews.EWS_On_Momentary_Button("Move", "Start", 80,25,15,"#000000","#AAAAAA");
    html_str2 += ews.EWS_On_Momentary_Button("NoMove", "Pause", 80,25,15,"#FFFFFF","#555555");
    html_str2 += "<br>\r\n";
    html_str3 += "<br>Scrolle Speed \r\n";
    html_str3 += ews.EWS_Canvas_Slider_T("Speed1",200,40,"#777777","#CCCCFF"); //CanvasスライダーはString文字列に２つまでしか入らない
    html_str3 += "<br>H_Size \r\n";
    html_str3 += ews.EWS_Canvas_Slider_T("Speed2",200,40,"#777777","#FFFFCC"); //CanvasスライダーはString文字列に２つまでしか入らない
    html_str4 += "<br>V_Size \r\n";
    html_str4 += ews.EWS_Canvas_Slider_T("Speed3",200,40,"#777777","#CCFFCC"); //CanvasスライダーはString文字列に２つまでしか入らない
    html_str4 += "<br><br>Text Color Picker \r\n";
    html_str4 += ews.Color_Picker(  0, 100, "#FFFFFF", "Color");
    html_str5 += "<br><br><br><br>\r\n";
    html_str5 += ews.EWS_WebSocket_Reconnection_Button2("WS-Reconnect", "grey", 200, 40, "black" , 17);
    html_str5 += "<br><br>\r\n";  
    html_str5 += ews.EWS_Close_Button2("WS CLOSE", "#bbb", 150, 40, "red", 17);
    html_str5 += ews.EWS_Window_ReLoad_Button2("ReLoad", "#bbb", 150, 40, "blue", 17);
    html_str5 += "</body></html>";
    //WebSocket ハンドシェイク関数
    ews.EWS_HandShake_main(3, CS_SD, HTM_head_file1, HTM_head_file2, dummy_file, dummy_file, LIP, html_str1, html_str2, html_str3, html_str4, html_str5, html_str6, html_str7);
 
    red = 0, green = 63, blue = 0;
    Status_Message( "Websocket SET", red, green, blue );
  }
}

// AquesTalk*******************************************************************
///////////////////////////
//
void Play(const char *koe)
{
  Serial.print("Play:");
  Serial.println(koe);

  int iret = CAqTkPicoF_SetKoe((const uint8_t*)koe, 50, 0xffffU);
  if(iret)  Serial.println("ERR:CAqTkPicoF_SetKoe");

  for(;;){
    int16_t wav[LEN_FRAME];
    uint16_t len;
    iret = CAqTkPicoF_SyntheFrame(wav, &len);
    if(iret) break; // EOD
    
    DAC_Write((int)len, wav);
  }
}

//i2s configuration 
const int i2s_num = 0; // i2s port number
i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
     .sample_rate = 24000,
     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
     .intr_alloc_flags = 0,
     .dma_buf_count = 4,
     .dma_buf_len = 384,
     .use_apll = 0
};

void DAC_Create()
{
  AqResample_Reset();

  i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin((i2s_port_t)i2s_num, NULL);
}

void DAC_Release()
{
  i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver 
}

// upsampling & write to I2S
int DAC_Write(int len, int16_t *wav)
{
  int i;
  for(i=0;i<len;i++){
    // upsampling x3
    int16_t wav3[3];
    AqResample_Conv(wav[i], wav3);

    // write to I2S DMA buffer
    for(int k=0;k<3; k++){
      uint16_t sample[2];
      uint16_t us = ((uint16_t)wav3[k])^0x8000U;  // signed -> unsigned data xxxxDA Only
      sample[0]=sample[1]=us; // mono -> stereo
      int iret = i2s_push_sample((i2s_port_t)i2s_num, (const char *)sample, 100);
      if(iret<0) return iret; // -1:ESP_FAIL
      if(iret==0) break;  //  0:TIMEOUT
    }
  }
  return i;
}
