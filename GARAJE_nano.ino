// EL TAMAÑO DEL KEY A USAR EN TODO EL PROGRAMA
#define KEY_SIZE    2*N_BLOCK
// MAXIMO 4 BLOQUES DE 16 BYTES, 64 BYTES DE BUFFER DE LECTURA
#define MAX_BUFFER  4*N_BLOCK
// MAXIMO TIEMPO QUE HAY PARA ENVIAR COMANDO, SI PASA ESE TIEMPO SE DESCONECTA
#define MAX_TIMEC   30000

#define BUZ_PIN     3
#define BUZ_PIN_    4
#define RL1_PIN     5
#define RL2_PIN     6
#define BLT_PIN     7

#define BTN_PIN     10
#define LED_PIN     11
#define LED_PIN_    12

// CAMBIAR EL VALOR Y QUITAR COMENTARIO PARA CAMBIAR EL NOMBRE
//#define BT_NAME     "111111111111"

#include "AES.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

SoftwareSerial BTserial(8, 9);

AES aes;

byte key[KEY_SIZE];
byte plain[MAX_BUFFER];
byte iv[N_BLOCK];
byte cipher[MAX_BUFFER];

uint8_t _status = 0;

char buff[MAX_BUFFER];
uint16_t nbuff=0;

// USANDO LA KEY, CREA UN IV
void CalculaIV() {
  iv[0]=key[17];
  iv[1]=key[14];
  iv[2]=key[7];
  iv[3]=key[11];
  iv[4]=key[12];
  iv[5]=key[30];
  iv[6]=key[1];
  iv[7]=key[27];
  iv[8]=key[2];
  iv[9]=key[7];
  iv[10]=key[18];
  iv[11]=key[5];
  iv[12]=key[21];
  iv[13]=key[3];
  iv[14]=key[25];
  iv[15]=key[8];
}

uint16_t getCommand(uint16_t time_out) {
  // LIMPIA BUFFER
  nbuff = 0;
  // LEE LA INFORMACION DEL BTSERIAL
  unsigned long mmm = millis();
  while(BTserial.available() || (millis()-mmm)<time_out) {
    if(BTserial.available()) {
      mmm = millis();
      buff[nbuff] = BTserial.read();
      if(nbuff<MAX_BUFFER) nbuff++;
    }
    yield();
  }
  return nbuff;
}

void Pitido(uint8_t num) {
  for(uint8_t ix=0;ix<num;ix++) {
    digitalWrite(BUZ_PIN, 1);
    digitalWrite(LED_PIN, 1);
    delay(200);
    digitalWrite(BUZ_PIN, 0);
    digitalWrite(LED_PIN, 0);
    delay(200);
  }
}

uint32_t btn_press=0;

void loop () {
  const long interval = 1000;
  static unsigned long previousMillis = 0;
  static unsigned long previousMillis_COMMAND;
  static uint8_t _status_count1 = 0;
  unsigned long currentMillis = millis();
  
  // SE ENCARGA DE PRENDER EL LED CADA CIERTO TIEMPO
  if(_status == 1) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      _status_count1 = (_status_count1+1) % 2;
      digitalWrite(LED_PIN, _status_count1 == 0);
      digitalWrite(BUZ_PIN, _status_count1 == 0);
    }
  }
  else if(_status == 2) {
    if(!digitalRead(BTN_PIN)) {
      if(btn_press==0) {
        btn_press = millis();
      }
      else if((millis()-btn_press)>5000) {
        Serial.println("resetting key");
        // HACE SONIDO POR UN SEGUNDO ENTERO
        digitalWrite(LED_PIN, 1);
        digitalWrite(BUZ_PIN, 1);
        delay(1000);
        digitalWrite(LED_PIN, 0);
        digitalWrite(BUZ_PIN, 0);
        // LIMPIA EEPROM
        key[0] = 0;
        EEPROM.write(0,key[0]);
        _status = 1;
        // APAGA MODULO BT
        digitalWrite(BLT_PIN, 0);
        delay(500);
        // RECONECTA MODULO BT
        ConectaBTModule();
        // POSIBLE USO EN UNA HORA O SOLTANDO EL BOTON
        btn_press = millis() + 3600000;
      }
    }
    else if(btn_press>0) {
      btn_press=0;
    }

    if (currentMillis - previousMillis >= 50 && _status_count1 == 0) {
      digitalWrite(LED_PIN, 0);
      _status_count1++;
    }
    // SOLO HACER ESTO CADA 1000 MS = 1 SEGUNDO
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      _status_count1 = (_status_count1+1) % 11;
      if(_status_count1 == 0)
        digitalWrite(LED_PIN, 1);
    }
  }

  if(_status==1) {
    if(getCommand(200)>0) {
      // TIENE QUE MANDAR LA CADENA DE INICIACION
      if(memcmp(buff,"Somos libres, seámoslo siempre.",31)!=0) {
        // OCURRIO UN ERROR SALIR
        Pitido(1);
        return;
      }
      Serial.print(F("new generated key = "));
      for(uint8_t ix=0;ix<KEY_SIZE;ix++) {
        if(random(2) == 1) {
          key[ix] = random(48,58);
        }
        else {
          key[ix] = random(65,91);
        }
        EEPROM.write(ix,key[ix]);
        BTserial.write(key[ix]);
        Serial.print((char)key[ix]);
      }
      Serial.println();
      _status = 2;

      Pitido(3);
    }
  }
  else if(_status == 2 || _status == 3) {
    if(_status == 3 && (millis()-previousMillis_COMMAND)>MAX_TIMEC) {
      _status = 2;
      // APAGA MODULO BT
      digitalWrite(BLT_PIN, 0);
      delay(500);
      // RECONECTA MODULO BT
      ConectaBTModule();
    }
    if(getCommand(200)==48) {

      CalculaIV();
      
      byte succ = aes.set_key (key, 256) ;
      succ = aes.cbc_decrypt (buff, plain, 3, iv) ;
      if(memcmp(&(plain[0]),"LOGIN",5)==0 && memcmp(&(plain[27]),"LOGIN",5)==0) {
        _status = 3;
        Pitido(2);
        BTserial.print("OK");
        Serial.println("LOGIN right");
        previousMillis_COMMAND = millis();
      }
      else if(_status==3 && memcmp(&(plain[0]),"SIREN",5)==0 && memcmp(&(plain[27]),"SIREN",5)==0) {
        BTserial.print("OK");
        Serial.println("command SIREN");
        digitalWrite(RL1_PIN, 0);
        digitalWrite(RL2_PIN, 1);
        previousMillis_COMMAND = millis();
      }
      else if(_status==3 && memcmp(&(plain[0]),"OPENG",5)==0 && memcmp(&(plain[27]),"OPENG",5)==0) {
        BTserial.print("OK");
        Serial.println("command OPENG");
        digitalWrite(RL1_PIN, 1);
        digitalWrite(RL2_PIN, 0);
        delay(500);
        digitalWrite(RL2_PIN, 1);
        previousMillis_COMMAND = millis();
      }
      else if(_status==3 && memcmp(&(plain[0]),"EXITG",5)==0 && memcmp(&(plain[27]),"EXITG",5)==0) {
        BTserial.print("OK");
        _status = 2;
        // APAGA MODULO BT
        digitalWrite(BLT_PIN, 0);
        delay(500);
        // RECONECTA MODULO BT
        ConectaBTModule();
      }
      aes.clean();
    }
    else if(nbuff==32) {
      if(memcmp(buff,"Somos libres, seámoslo siempre.",32)==0) {
        BTserial.print("OK");
        return;
      }
    }
  }
}

bool getKeyEEPROM() {
  for(uint8_t ix=0;ix<KEY_SIZE;ix++) {
    key[ix] = EEPROM.read(ix);
    if(!isAlphaNumeric(key[ix])) {
      return false;
    }
  }
  return true;
}

void ConectaBTModule() {
  uint8_t ccc=0;
  BTserial.flush();
  while(ccc==0) {
    digitalWrite(BLT_PIN, 1);
    delay(1500);
    BTserial.write("AT+VERSION");
    if(getCommand(3000)==0) {
      Serial.println("no detected module HC-06");
      digitalWrite(BLT_PIN, 0);
      delay(500);
    }
    else
      ccc=1;
  }
  #ifdef BT_NAME
    spintf(buff,"AT+NAME%s",BT_name);
    BTserial.write(buff);
    if(getCommand(3000)==0) {
      Serial.println("no detected module HC-06");
      digitalWrite(BLT_PIN, 0);
      delay(500);
    }
  #endif
}

void setup ()
{
  // DEFINE DIRECCION DE LOS PINES
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(BUZ_PIN_, OUTPUT);
  pinMode(RL1_PIN, OUTPUT);
  pinMode(RL2_PIN, OUTPUT);
  pinMode(BLT_PIN, OUTPUT);
  // DEFINE ESTADO INICIAL DE LOS PINES
  digitalWrite(LED_PIN, 0);
  digitalWrite(LED_PIN_, 0);
  digitalWrite(BUZ_PIN, 0);
  digitalWrite(BUZ_PIN_, 0);
  digitalWrite(RL1_PIN, 1);
  digitalWrite(RL2_PIN, 1);
  digitalWrite(BLT_PIN, 0);
  // INTENTA OBTENER UN SEED PARA EL RANDOM
  pinMode(A0, INPUT);
  randomSeed(analogRead(A0));
  // INICIA LOS SERIALES
  Serial.begin(115200);
  BTserial.begin(57600);
  // CONECTA EL MODULO BLUETOOTH
  ConectaBTModule();
  // ESCRIBE EL SERIAL LA VERSION HALLADA
  Serial.print("found module HC-06 ver = ");
  buff[nbuff]=0;
  Serial.println(&(buff[2]));
  // LEE KEY INICIAL Y DEFINE ESTADO INICIAL
  if(getKeyEEPROM()) {
    Serial.print("key found = ");
    for(uint8_t ix=0;ix<KEY_SIZE;ix++) {
      Serial.print((char)key[ix]);
    }
    Serial.println();
    _status = 2;
  }
  // Y ESTABLECE ESTADO SIN KEY
  else {
    Serial.println(F("no key found"));
    _status = 1;
  }
}
