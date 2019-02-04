// ==========================================================================================================================================================
// SISTEMA DE APERTURA DE PUERTA DE GARAGE (LIGHT TPMS)
// ==========================================================================================================================================================

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BUTTON_PIN          0
#define BUZZER_PIN          23
#define LED_PIN             21
#define RELAY1_PIN          19
#define RELAY2_PIN          18

char buff[256], buff_tmp[256], buff_tmp2[256];

// SI EL BOTON ESTA PRESIONADO, INICIALMENTE ASUMIR QUE NO ESTA PRESIONADO
uint8_t btn_mode=0;
// MODO DE EJECUCION
uint8_t sys_mode;
// TIPO DE COMANDO EN USO ACTUALMENTE
uint8_t comando=0;

BLECharacteristic *pChara;
BLEServer *pServer;

#define MIN_RND_NUMBER      20
#define MAX_RND_NUMBER      30

uint8_t rnd_tama;
uint32_t rnd_number[MAX_RND_NUMBER];

void printSerialHex(byte *cad, uint16_t ncad) {
  for (uint16_t ix=0; ix<ncad; ix++) {
    if(cad[ix]<16) 
      Serial.print("0");
    Serial.print(cad[ix],HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// ==========================================================================================================================================================
// LIBRERIAS DE ENCRIPTACION
// ==========================================================================================================================================================

#include "AES.h"

AES aes ;

char key[33];
byte my_iv[16];

void calc_iv() {
  my_iv[0] =(byte)key[17];
  my_iv[1] =(byte)key[14];
  my_iv[2] =(byte)key[7];
  my_iv[3] =(byte)key[11];
  my_iv[4] =(byte)key[12];
  my_iv[5] =(byte)key[30];
  my_iv[6] =(byte)key[1];
  my_iv[7] =(byte)key[27];
  my_iv[8] =(byte)key[2];
  my_iv[9] =(byte)key[7];
  my_iv[10]=(byte)key[18];
  my_iv[11]=(byte)key[5];
  my_iv[12]=(byte)key[21];
  my_iv[13]=(byte)key[3];
  my_iv[14]=(byte)key[25];
  my_iv[15]=(byte)key[8];
}

int Crypt(uint8_t data_n) {

  if(aes.set_key ((byte *)key, 256) == FAILURE) {
    return FAILURE;
  }
  calc_iv();

  int decodedLen = data_n;
  decodedLen = (decodedLen/N_BLOCK) + 1;

  if(aes.cbc_encrypt((byte *)buff, (byte *)buff_tmp, decodedLen, my_iv) == FAILURE) {
    return FAILURE;
  }

  aes.clean();

  return decodedLen * N_BLOCK;
}

int Decrypt(uint8_t data_n) {

  if(aes.set_key ((byte *)key, 256) == FAILURE) {
    return FAILURE;
  }
  calc_iv();

  int decodedLen = data_n;
  decodedLen = (decodedLen/N_BLOCK) + 1;

  if(aes.cbc_decrypt((byte *)buff, (byte *)buff_tmp, decodedLen, my_iv) == FAILURE) {
    return FAILURE;
  }

  aes.clean();

  return decodedLen * N_BLOCK;
}

// ==========================================================================================================================================================
// 
// ==========================================================================================================================================================

#include <Preferences.h>

Preferences preferences;

bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    // MODO RECEPCION DE KEY
    if(sys_mode == 0) {
      if (value.length() != 32) {
        pCharacteristic->setValue("");
        Serial.println("Key invalido...");
        return;
      }
      Serial.print("Nueva key : ");
      memset(key, 0, sizeof(key));
      for (uint16_t ix=0; ix<value.length(); ix++) {
        if(value[ix]<16) 
          Serial.print("0");
        Serial.print(value[ix],HEX);
        Serial.print(" ");
        key[ix] = value[ix];
      }
      Serial.println();
      // GRABA LA NUEVA KEY
      preferences.begin("DOOR", false);
      preferences.putString("key", key);
      preferences.end();
      // CAMBIA DE MODO Y CREA LA PRIMERA PREGUNTA
      sys_mode = 1;
      CreaPregunta(true);
      pCharacteristic->setValue(buff);
    }
    // MODO RECEPCION DE COMANDO
    else {
      for (uint16_t ix=0; ix<value.length(); ix++) {
        buff[ix] = value[ix];
      }
      uint16_t tm = Decrypt(value.length());

      /*Serial.print("Nuevo DATA : ");
      for (uint16_t ix=0; ix<tm; ix++) {
        if(buff_tmp[ix]<16) 
          Serial.print("0");
        Serial.print(buff_tmp[ix],HEX);
      }
      Serial.println();*/

      memset(buff_tmp2,0,sizeof(buff_tmp2));
      for(uint16_t ix=0;ix<rnd_tama;ix++) {
        memcpy((void *)&(buff_tmp2[ix*4]),(void *)&(rnd_number[ix]),sizeof(uint32_t));
      }
      for(uint16_t ix=0;ix<(rnd_tama*4);ix++) {
        buff[ix]=buff_tmp2[(rnd_tama*4)-ix-1];
      }
      if(!memcmp(&(buff_tmp[1]),buff,rnd_tama*4)) {
        // EJECUTA SEGUN buff_tmp[0]
        if(buff_tmp[0]==1) {
          Serial.println("** TOCANDO SIRENA **");
          comando=1;
          digitalWrite(RELAY1_PIN, LOW);
        }
        if(buff_tmp[0]==10) {
          Serial.println("** ABRIENDO PUERTA **");
          digitalWrite(RELAY1_PIN, HIGH);
          digitalWrite(RELAY2_PIN, LOW);
          digitalWrite(BUZZER_PIN, HIGH);
          delay(500);
          digitalWrite(RELAY2_PIN, HIGH);
          digitalWrite(BUZZER_PIN, LOW);
        }
      }

      CreaPregunta(false);
    }
  }
};

// ESTA CLASE SE ENCARGA DE SABER SI ESTAMOS CONECTADOS O NO
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Conectado");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Desconectado");
    // SI POR CASUALIDAD SE DESCONECTO Y ESTABA EN SIRENA, APAGAR SIRENA
    if(comando) {
      comando=0;
      digitalWrite(RELAY1_PIN, HIGH);
    }
  }
};

void CreaPregunta(bool recalc) {
  if(recalc)
    rnd_tama = MIN_RND_NUMBER + (esp_random()%(MAX_RND_NUMBER-MIN_RND_NUMBER));
  memset(buff,0,sizeof(buff));
  buff[0]=rnd_tama;
  for(int ix=0;ix<rnd_tama;ix++) {
    if(recalc) {
      rnd_number[ix] = esp_random();
      // EL BYTE 0X43 ES EL MARCADOR QUE USA EL PRORAMA PARA VER QUE TODO ESTA BIEN
      if(ix==0) {
        rnd_number[ix] &= 0xFFFFFF00;
        rnd_number[ix] |= 0x00000043;
      }
    }
    memcpy((void *)&(buff[ix*4+1]),(void *)&(rnd_number[ix]),sizeof(uint32_t));
  }
  Serial.println("Creando pregunta...");
  /*for(int ix=0;ix<(rnd_tama*4);ix++) {
    if(buff[ix]<16)
      Serial.print("0");
    Serial.print((uint8_t)buff[ix+1],HEX);
  }
  Serial.println();*/
  uint16_t tm = Crypt(sizeof(uint32_t)*MAX_RND_NUMBER);
  pChara->setValue((uint8_t *)buff_tmp, tm);
}

void IRAM_ATTR isr() {
  btn_mode = 1;
  if(digitalRead(BUTTON_PIN))
    btn_mode = 0;
}

void setup() {
  // DEFINE LOS PINES QUE USAREMOS
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  // LOS VALORES INICIALES DE LOS PINES
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  // ACTUALIZA LA VARIABLE BTN_MODE CON UNA INTERRUPCION
  attachInterrupt(BUTTON_PIN, isr, CHANGE);
  // INICIA SERIAL
  Serial.begin(115200);
  // DEFINE EL NOMBRE DEL DISPOSITIVO
  uint64_t chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
  sprintf(buff_tmp, "DOOR_%04X%08X",(uint16_t)(chipid>>32),(uint32_t)chipid);
  // IMPRIME INFORMACION EN EL SERIAL
  Serial.printf("Iniciando Servicio BLE : %s", buff_tmp);
  Serial.println();
  // INICIA SERVIDOR GATT
  BLEDevice::init(buff_tmp);
  pServer = BLEDevice::createServer();
  // DEFINE LOS CALLBACKS DEL SERVIDOR
  pServer->setCallbacks(new MyServerCallbacks());
  // CREA EL SERVICIO
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // CREA LA CARACTERISTICA
  pChara = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  // DEFINE LOS CALLBACKS DE LA CARACTERISTICA
  pChara->setCallbacks(new MyCallbacks());
  // LEE LAS PREFERENCIAS SOBRE EL KEY
  preferences.begin("DOOR", false);
  memset(key,0,sizeof(key));
  preferences.getString("key", key, sizeof(key));
  preferences.end();
  // SI NO HAY KEY SIGNIFICA QUE INICIA EN MODO NUEVO
  if(strlen(key) != 32) {
    sys_mode=0;
    pChara->setValue("");
    Serial.println("Iniciando modo NUEVO");
  }
  // SI HAY KEY SIGNIFICA QUE ENTRA EN MODO COMANDO
  else {
    Serial.println("Iniciando modo COMANDO");
    printSerialHex((uint8_t *)key, 32);
    sys_mode=1;
  }
  // INICIA POR FIN EL SERVICIO
  pService->start();
  // COMIENZA A ENVIAR LOS ADVERTISING
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  // EFECTO SONORO DE INICIO
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

// CONTADORES
uint16_t loop_count=0;
uint8_t btn_count=0, sound_count=0;

void loop() {
  delay(100);
  // SI SE PRESIONA MAS DE 5 SEGUNDOS EL BOTON SIGNIFICA QUE ELIMINA LA KEY
    if(btn_mode == 1) {
      if(btn_count<60)
        btn_count++;
      if(btn_count==50) {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        sys_mode=0;
        pChara->setValue("");
        preferences.begin("DOOR", false);
        preferences.putString("key", "");
        preferences.end();
        Serial.println("Eliminando key, entrando en modo NUEVO...");
        delay(1000);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        return;
      }
    }
    else if(btn_count>0)
      btn_count = 0;
  // SI ESTA EN MODO NUEVO, HACE PITIDO
  if(sys_mode == 0) {
    if(sound_count==0) {
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
    }
    if(sound_count==2) {
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
    }
    sound_count = (sound_count + 1) % 20;
  }
  else {
    if(sound_count==0)
      digitalWrite(LED_PIN, HIGH);
    if(sound_count==1)
      digitalWrite(LED_PIN, LOW);
    if(deviceConnected)
      sound_count = (sound_count + 1) % 10;
    else
      sound_count = (sound_count + 1) % 100;
  }
  // SI NO ESTA CONECTADO CAMBIA LA PREGUNTA CADA 5 MINUTOS = 300 SEGUNDOS
  if (!deviceConnected) {
    if(loop_count==0 && sys_mode==1) {
      CreaPregunta(true);
    }
    loop_count = (loop_count + 1) % 3000;
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("(restart advertising)");
    loop_count=0;
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}
