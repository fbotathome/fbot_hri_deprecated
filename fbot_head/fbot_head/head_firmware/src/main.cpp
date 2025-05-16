#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <map>
#include <Preferences.h>

//functions declarations
std::map<String, Servo> configureMotors(JsonObject motors_config);

int getCommand(JsonObject doc);

JsonDocument stringToJsonObject(String str);

String write_to_motors(JsonObject motors_angles);

void clearMotors();

//global variables
std::map<String, Servo> motors;

enum commands : int {
  ERROR, CONFIG, WRITE_ANGLES
};

Preferences preferences;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  preferences.begin("storage", false);
  
  String stored_config = preferences.getString("json_config", "");

  if(!stored_config.isEmpty()){

    clearMotors();
    JsonDocument json_doc = stringToJsonObject(stored_config);
    JsonObject json_obj = json_doc.as<JsonObject>();
    motors = configureMotors(json_obj);

  }

}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial.available()==1){ //FAZER CHAVE VALOR COM COMANDO NO JSON PARA QUANDO FOR CONFIGURAÇÃO E QUANDO FOR ESCRITA DE VALORES!!!

    //Serial.print("LOOP!!");
    String string_json = Serial.readString();

    JsonDocument json_doc = stringToJsonObject(string_json);

    JsonObject json_obj = json_doc.as<JsonObject>();

    int cmd = getCommand(json_obj);

    String response;

    //Serial.print("command = ");
    //Serial.print(cmd);

    switch (cmd)
    {
    case ERROR: //Command not found or Json not structured correctly
      response = "{\"response\": \"error\"}";
      
      break;
    
    case CONFIG: //Command for instanciating motors

      preferences.putString("json_config", string_json);

      clearMotors(); // Clear the existing motors map
      
      motors = configureMotors(json_obj);

      response = "{\"response\": \"success\", \"message\": [";
      for (auto it = motors.begin(); it != motors.end(); ++it) {
        response += "\"" + it->first + "\"";
        if (std::next(it) != motors.end()) {
          response += ", ";
        }
      }
      response += "]}";
      
      break;

    case WRITE_ANGLES:

      //Serial.println(cmd);
      //Serial.println(WRITE_ANGLES);

      response = write_to_motors(json_obj);
      
      break;

    default:

      response = "{\"response\": \"command not found\"}";

      break;
    }

    Serial.print(response);


  }
}

int getCommand(JsonObject doc){

  if (doc.isNull()) {
    //Serial.println("JsonObject é nulo!");
    return ERROR; // Retorna um valor de erro
  }

  int command_value = 0;

  if(doc["cmd"].is<JsonVariant>()){
    command_value = doc["cmd"].as<int>();
  }

  //Serial.println(command_value);
  doc.remove("cmd");
  return command_value;

}

JsonDocument stringToJsonObject(String str){

  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, str);

  if (error) {
    //Serial.print("Erro ao desserializar JSON: ");
    //Serial.println(error.c_str());
    
  } else {
    //Serial.println("JSON desserializado com sucesso!");
  }

  return doc;

}

std::map<String, Servo> configureMotors(JsonObject motors_config) {
  std::map<String, Servo> motors_dict;

  if(motors_config["cmd"].is<JsonVariant>()){
    motors_config.remove("cmd");
  }

  for (JsonPair key_value : motors_config) {
    const char* motor_name = key_value.key().c_str();
    int motor_pin = key_value.value().as<int>();

    Servo* motor = new Servo(); // Aloca dinamicamente o objeto Servo
    motor->attach(motor_pin);

    motors_dict[motor_name] = *motor; // Armazena o objeto no mapa
  }

  return motors_dict;
}

String write_to_motors(JsonObject motors_angles) { // Testar movimentação dos motores

  for (JsonPair key_value : motors_angles) {

    String motor_name = key_value.key().c_str(); 
    int motor_angle = key_value.value().as<int>();   

    Servo motor = motors[motor_name];

    if (!motor.attached()) {
      //Serial.print("Erro: Motor ");
      //Serial.print(motor_name);
      //Serial.println(" não está anexado a nenhum pino.");
      return "{\"response\": \"error\", \"message\": \"Motor not attached: " + motor_name + "\"}";
    }

    //Serial.print("Movendo motor: ");
    //Serial.println(motor_name);
    //Serial.print("Angulo: ");
    //Serial.println(motor_angle);

    motor.write(motor_angle);
  }

  return "{\"response\": \"success\", \"message\": \"Motors moved successfully\"}";
}

void clearMotors() {
  for (auto& motor_pair : motors) {
    Servo* motor_ptr = &motor_pair.second; 
    motor_ptr->detach();
  }
  motors.clear(); 
}