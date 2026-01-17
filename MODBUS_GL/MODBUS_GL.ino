/*
NOME:GABRIEL LEMOS
PLACA: ESP32 DEV MODULE

APRESENTAÇÃO: Usando um esp32 e um conversor CH340, enviei dados através da COM4 pelo conversor para o esp32, o qual usava para debugar pela COM3. 
Mantive uns prints das tentativas de debugar meu código, pois no inicio tive bastante dificuldade de entender.*/
//Variaveis globais
uint16_t slv_ad = 0x01;
uint16_t crc_calculado;
uint16_t crc_recebido;
const int n_coils = 10;
const int n_registers = 3;
uint16_t response[8];
uint8_t buffer_modbus[8];
int coils[n_coils] =  {2,21,19,18,22,23,12,13,26,27};

uint16_t temperature = 255;
uint16_t current = 10;
uint16_t voltage = 220;

//IMPLEMENTAÇÃO DA ESTRUTURA PARA MODBUS, IMPLEMENTANDO ERROS.
// Definições Modbus
#define FN_READ_COILS 0x01
#define FN_READ_HOLDING_REGISTERS 0x03
#define FN_WRITE_SINGLE_COIL 0x05
#define FN_WRITE_SINGLE_REGISTER 0x06
#define FN_WRITE_MULTIPLE_COILS 0x0F
#define FN_WRITE_MULTIPLE_REGISTERS 0x10

//Holding registers analisados
uint16_t *HOLDING_REGISTERS[3] = {
  &temperature,
  &voltage,
  &current
};
// Códigos de exceção Modbus
typedef enum {
    MODBUS_OK = 0x00,
    MODBUS_ILLEGAL_FUNCTION = 0x01,
    MODBUS_ILLEGAL_DATA_ADDRESS = 0x02,
    MODBUS_ILLEGAL_DATA_VALUE = 0x03
} ModbusExceptionCode;

ModbusExceptionCode errorModbus;

// Protótipos de funções
void Modbus_ProcessBuffer(uint8_t *buffer, uint16_t length);
ModbusExceptionCode Modbus_ReadCoils(void);
ModbusExceptionCode Modbus_ReadHoldingRegisters(void);
ModbusExceptionCode Modbus_WriteSingleCoil(void);
ModbusExceptionCode Modbus_WriteSingleRegister(void);
ModbusExceptionCode Modbus_WriteMultipleCoils(void);
ModbusExceptionCode Modbus_WriteMultipleRegisters(void);
void Modbus_ErrorResponse(uint8_t FN_CODE);


//Implementação das funções modbus.

ModbusExceptionCode Modbus_WriteMultipleRegisters() {
  uint16_t start_address = (buffer_modbus[2] << 8) | buffer_modbus[3];
  uint16_t quantity = (buffer_modbus[4] << 8) | buffer_modbus[5];
  if(quantity < 1 || (quantity + start_address) > n_registers) {
    return MODBUS_ILLEGAL_DATA_ADDRESS;
  }
  for (uint16_t i = 0; i < quantity; i++) {
    if((start_address + i) == 0) {
      return MODBUS_ILLEGAL_DATA_ADDRESS; //Se algum endereço for igual a zero, retorna erro
    }
  }
  for (uint16_t i = 0; i < quantity; i++) {
        uint16_t data_index = 7 + i * 2; //os dados começam no byte 7 do modbusBuffer, como cada registrador ocupa 2 bytes...
        uint16_t register_value = (buffer_modbus[data_index] << 8) | buffer_modbus[data_index + 1];
        *HOLDING_REGISTERS[start_address + i] = register_value;
    }
  //Montando o frame
  uint8_t response[8] = {0};
  response[0] = slv_ad;
  response[1] = FN_WRITE_MULTIPLE_REGISTERS;
  response[2] = buffer_modbus[2];
  response[3] = buffer_modbus[3];
  response[4] = buffer_modbus[4];
  response[5] = buffer_modbus[5];
  uint16_t crc = calculaCRC(response,6);
  response[6] = crc & 0xFF;
  response[7] = (crc >> 8);

  Serial2.write(response, 8);
  return MODBUS_OK;
  }
ModbusExceptionCode Modbus_WriteMultipleCoils() {
    uint16_t start_address = (buffer_modbus[2] << 8) | buffer_modbus[3];
    uint16_t quantity = (buffer_modbus[4] << 8) | buffer_modbus[5];

    if (start_address + quantity > n_coils || quantity < 1 || quantity > 0x07B0) {
        return MODBUS_ILLEGAL_DATA_ADDRESS;
    }

    uint8_t byte_count = buffer_modbus[6];
    uint8_t expected_byte_count = (quantity / 8) + ((quantity % 8) ? 1 : 0);
    if (byte_count != expected_byte_count) {
        return MODBUS_ILLEGAL_DATA_VALUE;
    }

    for (uint16_t i = 0; i < quantity; i++) {
        uint8_t byte_index = 7 + (i / 8);
        uint8_t bit_index = i % 8;
        uint8_t bit_value = (buffer_modbus[byte_index] >> bit_index) & 0x01;
        digitalWrite(coils[start_address + i], bit_value ? HIGH : LOW);
    }

    uint8_t response[8];
    response[0] = slv_ad;
    response[1] = 0x0F;
    response[2] = buffer_modbus[2];
    response[3] = buffer_modbus[3];
    response[4] = buffer_modbus[4];
    response[5] = buffer_modbus[5];

    uint16_t crc = calculaCRC(response,6);
    response[6] = crc & 0xFF;
    response[7] = (crc >> 8);

    Serial2.write(response, 8);
    return MODBUS_OK;
}
ModbusExceptionCode Modbus_WriteSingleRegister() {
  //Quero escrever em um espaço de memoria do esp32, para isso usaremos essa função
  uint16_t register_address = (buffer_modbus[2] << 8) | buffer_modbus[3]; //O primeiro é byte MSB e o segundo é o byte LSB, fazendo um or obtenho um valor de 16 bits
  //Ou 2 bytes
  uint16_t register_value = (buffer_modbus[4] << 8) | buffer_modbus[5];
  if (register_address >= n_registers || register_address == 0 ) {
    return MODBUS_ILLEGAL_DATA_ADDRESS;
  }
  if (register_value > 0xFFFF) {
    return MODBUS_ILLEGAL_DATA_VALUE;
  }
  *HOLDING_REGISTERS[register_address] = register_value;

  //Montagem do frame de resposta

  uint8_t response[8] = {0};

  response[0] = slv_ad;
  response[1] = FN_WRITE_SINGLE_REGISTER;
  response[2] = buffer_modbus[2];
  response[3] = buffer_modbus[3];
  response[4] = buffer_modbus[4];
  response[5] = buffer_modbus[5];
  uint16_t crc = calculaCRC(response, 6);
  byte crcLow = crc & 0xFF; //Cada bit do crc 
  byte crcHigh = (crc >> 8);
  response[6] = crcLow;
  response[7] = crcHigh;
  Serial2.write(response,8);
  return MODBUS_OK;
}
ModbusExceptionCode Modbus_WriteSingleCoil() {
  uint16_t output_address = (buffer_modbus[2] << 8) | buffer_modbus[3];
  uint16_t output_state = (buffer_modbus[4] << 8) | buffer_modbus[5]; // 0x0000 OFF, FF00 ON
  if (output_address > n_coils) {
    return MODBUS_ILLEGAL_DATA_ADDRESS;
  }
  if (output_state == 0xFF00) {
    digitalWrite(coils[output_address],1);
  }
  else if(output_state == 0x0000){ 
    digitalWrite(coils[output_address],0);
  }
  else {
    return MODBUS_ILLEGAL_DATA_VALUE;
  }
  uint8_t response[8] = {0};
  response[0] = slv_ad;
  response[1] = 0x05;
  //Como só posso enviar 1 byte por vez no frame, não posso usas output_state e nem o output_adress
  response[2] = buffer_modbus[2];
  response[3] = buffer_modbus[3];
  response[4] = buffer_modbus[4];
  response[5] = buffer_modbus[5];
  uint16_t crc = calculaCRC(response, 6);
  byte crcLow = crc & 0xFF; //Cada bit do crc 
  byte crcHigh = (crc >> 8);
  response[6] = crcLow;
  response[7] = crcHigh;
  Serial2.write(response,8);
  return MODBUS_OK;
}
ModbusExceptionCode Modbus_ReadHoldingRegisters() {
  uint16_t value = 0;
  uint16_t regHigh = 0;
  uint16_t regLow = 0;
  //No modbus podemos ler 3 registradores por vez 
  //Vamos ter que ler a quantidade de coils, ou saídas, respeitando o endereço inicial e final
  uint16_t start_address = (buffer_modbus[2] << 8) | buffer_modbus[3]; //O primeiro é byte MSB e o segundo é o byte LSB, fazendo um or obtenho um valor de 16 bits
  //Ou 2 bytes
  uint16_t quantity = (buffer_modbus[4] << 8) | buffer_modbus[5]; //Retorna 2 bytes, mesma forma do anterior 0xabcd.
   if((start_address + quantity) > n_registers) {
    Serial.println("MODBUS_ILLEGAL_ADDRESS");
    Serial.println("parou holding ad");
    return MODBUS_ILLEGAL_DATA_ADDRESS;
  }
  if(quantity < 0x0001 || quantity > 0x07D0 ) {
    Serial.println("MODBUS_ILLEGAL_VALUE");
    Serial.println("parou holding value");
    return MODBUS_ILLEGAL_DATA_VALUE;
  }
  //Como cada registrador tem 16 bits, ou seja, 2 bytes, para cada registrador vamos ocupar 2 byte_count

  uint16_t byte_count = quantity * 2;

  //Criamos um vetor de 8 bytes para armazenar as respostas, esse vetor tem que ter tamanho suficiente para suportar toda ela.

  uint8_t response[256] = {0};

  //Montando o frame de resposta

  response[0] = slv_ad;
  response[1] = 0x03;
  response[2] = byte_count;
  
  //Definição do estado dos registradores, como os holding registers estão armazenados em um endereço, utilizo um ponteiro para pegar esses valores
  for(uint16_t i = 0; i < quantity; i++) {
    value = *(HOLDING_REGISTERS[start_address + i]); //Pego o valor do holding register a partir de start_adress
    regHigh = (value >> 8) & 0xFF; // Desloco o frame 8 bits para a direita, ficando apenas os bits mais significativos
    regLow = value & 0xFF;
    response[3 + (i*2)] = regHigh;
    response[4 + (i*2)] = regLow;
  }
  uint16_t crc = calculaCRC(response, 3 + byte_count);
  byte crcLow = crc & 0xFF; //Cada bit do crc 
  byte crcHigh = (crc >> 8);
  response[3 + byte_count] = crcLow;
  response[4 + byte_count] = crcHigh;
  // Serial.println(byte_count,HEX);
    // Envia a resposta pela porta Serial2
//   for (int i = 0; i < (5 + byte_count); i++) {
//     Serial2.write(response[i]);
// }
  // for (int i = 0; i < 5 + byte_count; i++) {
  //   Serial.print("resposta:");
  //   Serial.println(response[i]);
  // }
  Serial2.write(response, 5 + byte_count);
  return MODBUS_OK;

}
ModbusExceptionCode Modbus_ReadCoils() {

  //Vamos ter que ler a quantidade de coils, ou saídas, respeitando o endereço inicial e final
  uint16_t start_address = (buffer_modbus[2] << 8) | buffer_modbus[3]; //O primeiro é byte MSB e o segundo é o byte LSB, fazendo um or obtenho um valor de 16 bits
  //Ou 2 bytes
  uint16_t quantity = (buffer_modbus[4] << 8) | buffer_modbus[5]; //Retorna 2 bytes, mesma forma do anterior 0xabcd.
  //Quero definir como 5 saídas e ja era. Vou ler do 2 até o 10. 10 = 0X0A
  if((start_address + quantity) > n_coils) {
    Serial.println("MODBUS_ILLEGAL_ADDRESS");
    Serial.print("parou aqui");
    return MODBUS_ILLEGAL_DATA_ADDRESS;
  }

  if(quantity < 0x0001 || quantity > 0x07D0 ) {
    Serial.println("MODBUS_ILLEGAL_VALUE");
    return MODBUS_ILLEGAL_DATA_VALUE;
  }

  //Agora tenho que montar a resposta para devolver para o mestre.
  uint8_t byte_count = (quantity + 7)/8; //Arredonda pra cima o numero de bytes necessário para traduzir o numero de saídas
  uint8_t response[5 + byte_count] = {0}; //Máximo de 8 bytes
  response[0] = slv_ad; //Padrão
  response[1] = 0x01;   //Endereço da função readCoil
  response[2] = byte_count;

  //Agora tenho que ler essas saídas e armazenar na memoria, tenho que fazer isso em 3 bytes, para formar os 6 bytes de dados totais.

  for(uint16_t i = 0; i < quantity; i++){
    uint8_t coil_state = digitalRead(coils[start_address + i]);
     response[3 + (i / 8)] |= coil_state << (i % 8); // um byte pode armazenar o estado de 8 coils,
  }
  Serial.print("ENTROU AQUI:   ");
  Serial.println(response[3],BIN);

  uint16_t crc = calculaCRC(response, 3 + byte_count);
  byte crcLow = crc & 0xFF; //Cada bit do crc 
  byte crcHigh = (crc >> 8);
  response[3 + byte_count] = crcLow;
  response[4 + byte_count] = crcHigh;
  // Serial.println("BYTE COUNT: ");
  // Serial.println(byte_count);
  // Serial.print("resposta 4: ");
  // Serial.println(response[4],HEX);
  // Serial.print("CRC HIGH:  ");
  // Serial.println(crcHigh,HEX);
  // Serial.print("CRC LOW:  ");
  // Serial.println(crcLow,HEX);
  // for ( int i = 0; i < n_coils; i++ ) {
  //   Serial.print("Resposta:"); // Verifica a resposta.
  //   Serial.println(response[i],HEX);
  // }
  // Envia a resposta pela porta Serial2
  for (int i = 0; i < (5 + byte_count); i++) {
  Serial2.write(response[i]);
}
  return MODBUS_OK;
}
void Modbus_ErrorResponse(uint8_t FN_CODE) {
  uint8_t response_error[5] = {0}; //Pacote a ser rebatido em caso de erro modbus
  response_error[0] = slv_ad;
  response_error[1] = FN_CODE | 0x80; // 10000000, ativando o bit MSB
  response_error[2] = errorModbus;
  uint16_t crc_erro = calculaCRC(response_error,3);
  response_error[3] = crc_erro & 0xFF;
  response_error[4] = (crc_erro >> 8);
  Serial2.write(response_error,5);
}
void Modbus_ProcessBuffer(uint8_t *buffer,uint16_t length) {
  //Armazena o frame em outra variavel global para poder ser lida depois
  for (int i = 0; i < length; i++) {
    digitalWrite(coils[3],1);
    buffer_modbus[i] = buffer[i];
  }
  
  errorModbus = MODBUS_OK; //Inicia o modbus tudo ok
  uint8_t fn_default = buffer_modbus[1];
  switch(buffer_modbus[1]) {
    case FN_READ_COILS:
      errorModbus = Modbus_ReadCoils();
      if(errorModbus != MODBUS_OK) {
        Modbus_ErrorResponse(FN_READ_COILS);
      }
    break;
    case FN_READ_HOLDING_REGISTERS:
      errorModbus = Modbus_ReadHoldingRegisters();
      if(errorModbus != MODBUS_OK) {
        Modbus_ErrorResponse(FN_READ_HOLDING_REGISTERS);
      }
    break;
    case FN_WRITE_SINGLE_COIL:
      errorModbus = Modbus_WriteSingleCoil();
      if(errorModbus != MODBUS_OK) {
        Modbus_ErrorResponse(FN_WRITE_SINGLE_COIL);
      }
    break;
    case FN_WRITE_SINGLE_REGISTER:
      errorModbus = Modbus_WriteSingleRegister();
      if(errorModbus != MODBUS_OK) {
        Modbus_ErrorResponse(FN_WRITE_SINGLE_REGISTER);
      }
    case FN_WRITE_MULTIPLE_COILS:
      errorModbus = Modbus_WriteMultipleCoils();
      if(errorModbus != MODBUS_OK) {
        Modbus_ErrorResponse(FN_WRITE_MULTIPLE_COILS);
      }
    break;
    default:
      Modbus_ErrorResponse(fn_default);
  }
}
uint16_t valid_code_func[3] = {0x01, 0x05, 0x0F};
// REG_WRITE(GPIO_ENABLE_REG,BIT2 + BIT4); //GPIO 2 E GPIO 4 como saída. # Não é necessario implementar
//

uint16_t calculaCRC(uint8_t *c_buffer,uint8_t c_max) {
    uint16_t i_CRC = 0xFFFF;
    for (uint8_t c_i = 0; c_i < c_max; c_i++) {
        i_CRC ^= c_buffer[c_i];
        for (uint8_t c_j = 0; c_j < 8; c_j++) {
            if (i_CRC & 0x0001) {
                i_CRC >>= 1;
                i_CRC ^= 0xA001;
            } else {
                i_CRC >>= 1;
            }
        }
    }
    return i_CRC;
}



bool fn_validCodeFunction(uint16_t data) {
  int qtd = sizeof(valid_code_func) / sizeof(valid_code_func[0]); // Correto: número de elementos
  for (int i = 0; i < qtd; i++) {  // "<" e não "<="
    if (data == valid_code_func[i]) {
      return true;
    } 
  }
  return false;
}

//Implementar funções
void write_single_coil(byte frame[8]) {
  return;
}

void main_modbus() {
   if (Serial2.available() > 0) { // Verifica se tem algo no serial 2
    if (Serial2.peek() == 0x01 || Serial2.peek() == 0x02) {  // Espera que o byte inicial seja x01
      if (Serial2.available() >= 8) {  // só lê quando tem o frame inteiro
        byte frame[8];
        Serial2.readBytes(frame, 8); //Se tem, le os 8 bytes
          // Calcula CRC apenas dos primeiros 6 bytes
        // uint16_t crc = calculaCRC(frame, 6);
        // byte crcLow = crc & 0xFF; //Cada bit do crc 
        // byte crcHigh = (crc >> 8);
        Serial.print("Frame + CRC: ");
        for (int i = 0; i < 8; i++) {
            // Calcula CRC apenas dos primeiros 6 bytes
          
          Serial.print(frame[i] < 0x10 ? "0" : "");
          Serial.print(frame[i], HEX);
          //Assim que se manda a mensagem de volta
          // Serial2.write(frame[i]);
          //====================
          Serial.print(" ");
        }
      
        //Começo da implementação.
        Modbus_ProcessBuffer(frame,8); //Inicia tudo
        if(fn_validCodeFunction(frame[2])) {
          Serial.println("É valido");
        }
        else {
          Serial.println("É INVALIDO");}
        Serial.println();


      
        // if (frame[0] == 0x01) {
        //   digitalWrite(led, HIGH);
        //   frame_pass = 1;
        // } else {
        //   digitalWrite(led, LOW);
        // }
      }
    } else {
      Serial2.read();  // descarta byte fora de posição até achar o começo certo
    }
  }
  
}

void fn_verifica_ad(uint16_t data) {
  if (data == slv_ad) {
    Serial.println("Escravo encontrado!");
  }
  else {
    Serial.println("...");
  }
}
#define led 2
void setup() {
  // Inicia a serial para debug (USB)
  Serial.begin(115200);
  for (int i = 0; i < n_coils; i++) {
    pinMode(coils[i],OUTPUT);
    digitalWrite(coils[i],0); // Inicializo todas as coils com 0
  }
  // Pausa até a serial estar sssssronta (opcional, mas útil para depuração)
  while (!Serial) {
    delay(10); // Evita travar se não houver conexão USB
  }

  // Configura a serial para Modbus (UART2 - GPIO16=RX, GPIO17=TX)
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Baud rate do Modbus RTU
  
  Serial.println("ESP32 inicializado!");
  Serial.println("Aguardando dados na Serial2...");

  //Inicializo sem erros 
  
  memset(buffer_modbus, 0, sizeof(buffer_modbus)); // Zera o vetor buffer_modbus da memoria
  errorModbus = MODBUS_OK;
}

void loop() {
  main_modbus();
 
}
