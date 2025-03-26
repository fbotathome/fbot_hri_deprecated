#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <map>

// put function declarations here:
int myFunction(int, int);
std::map<String, Servo> create_motors();

std::map<String, Servo> motors_dict;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  std::map<String, Servo> motors = create_motors();

  // Testar movimentação dos motores
  for (auto& pair : motors) { // Removido o 'const'
    Serial.print("Movendo motor: ");
    Serial.println(pair.first); // Nome do motor
    pair.second.write(90);      // Mover o motor para 90 graus
}

}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial.available()==1){ //FAZER CHAVE VALOR COM COMANDO NO JSON PARA QUANDO FOR CONFIGURAÇÃO E QUANDO FOR ESCRITA DE VALORES!!!

    Serial.print("available: \n");
    Serial.print(Serial.readString());
  }
}

std::map<String, Servo> create_motors() {

  JsonDocument motors_config;

  int configured = 0;

  int motors_num = 0;

  while (!configured) {
    if (Serial.available()) {
      Serial.println("available: \n");

      String input = Serial.readString();

      Serial.println(input);
      Serial.println("");

      DeserializationError error = deserializeJson(motors_config, input);

      if (error) {
        Serial.print("Erro ao desserializar JSON: ");
        Serial.println(error.c_str());
      } else {
        Serial.println("JSON desserializado com sucesso!");

        motors_num = motors_config.size();

        configured=1;

      }
    }
  }

  Servo motor[motors_num];

  int index=0;
  // Iterar sobre as chaves e valores do JSON
  for (JsonPair key_value : motors_config.as<JsonObject>()) {
    
    const char* motor_name = key_value.key().c_str(); // Nome da chave no JSON
    int motor_pin = key_value.value().as<int>();    // Valor associado (pino do motor)


    Serial.print("Chave: ");
    Serial.print(key_value.key().c_str());

    motor[index].attach(motor_pin);

    motors_dict[motor_name] = motor[index];

    Serial.print(" | Valor: ");
    Serial.println(key_value.value().as<String>());

    index++;
  }

  return motors_dict;

}


// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}