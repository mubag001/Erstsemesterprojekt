#include <ESP32Servo.h> 
 
////////////////////////////////////////////// 
// RemoteXY-Bibliothek einbinden // 
////////////////////////////////////////////// 
 
#define REMOTEXY_MODE__ESP32CORE_BLE 
#include <BLEDevice.h> 
#include <RemoteXY.h> 
 
// RemoteXY-Verbindungseinstellungen 
#define REMOTEXY_BLUETOOTH_NAME "CyberTruckLite" 
#define REMOTEXY_ACCESS_PASSWORD "2004" 
 
// RemoteXY-Konfiguration 
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] =  
{  
  255,5,0,3,0,62,0,16,200,0,5,32,2,24,37,37,31,213,31,10, 
  48,48,47,10,10,4,36,31,79,78,0,31,79,70,70,0,1,8,79,41, 
  13,13,213,31,75,105,99,107,0,1,8,49,33,9,9,213,31,82,101,115, 
  101,116,0,65,112,91,1,7,7  
}; 
 
struct { 
  int8_t joystick_x; // von -100 bis 100 
  int8_t joystick_y; // von -100 bis 100 
  uint8_t pushSwitch_1; // =1, wenn Zustand EIN ist, sonst =0 
  uint8_t button_1; // =1, wenn Knopf gedrückt, sonst =0 
  uint8_t button_2; // =1, wenn Knopf gedrückt, sonst =0 
  uint8_t led_2_r; // =0..255 LED-Rot-Helligkeit 
  uint8_t led_2_g; // =0..255 LED-Grün-Helligkeit 
  uint8_t led_2_b; // =0..255 LED-Blau-Helligkeit 
  uint8_t connect_flag; // =1, wenn Verbindung hergestellt, sonst =0 
} RemoteXY; 
#pragma pack(pop) 
 
////////////////////////////////////////////// 
// Motor-, Sensor- und Servo-Pins // 
////////////////////////////////////////////// 
int Motor_links_A = 3;  // Pin für Motor links A 
int Motor_links_B = 2;  // Pin für Motor links B 
int Motor_rechts_A = 18; // Pin für Motor rechts A 
int Motor_rechts_B = 19; // Pin für Motor rechts B 
 
int frontSensorPin = 0;  // Analogpin für QTR-1A-Sensor (vorne) 
int rearSensorPin = 1;   // Analogpin für QTR-1A-Sensor (hinten) 
 
int frontSensorValue = 0;  // Variable zum Speichern des Frontsensorwerts 
int rearSensorValue = 0;   // Variable zum Speichern des Rücksensorwerts 
int threshold = 4000;      // Schwellenwert zur Erkennung einer schwarzen Linie 
bool previousButton1State = false; 
 
int servoPin = 5;          // Pin für den SG90-Servo 
Servo kickServo;           // Servo-Objekt 
 
////////////////////////////////////////////// 
// Setup-Funktion // 
////////////////////////////////////////////// 
void setup() { 
  RemoteXY_Init(); 
  Serial.begin(115200); 
 
  // Setze die Motorpins als Ausgänge 
  pinMode(Motor_links_A, OUTPUT); 
  pinMode(Motor_links_B, OUTPUT); 
  pinMode(Motor_rechts_A, OUTPUT); 
  pinMode(Motor_rechts_B, OUTPUT); 
 
  // Richten Sie die Sensorpins ein 
  pinMode(frontSensorPin, INPUT); 
  pinMode(rearSensorPin, INPUT); 
 
  // Initialisiere den Servo 
  kickServo.attach(servoPin); 
  kickServo.write(0); // Sicherstellen, dass der Servo in der Ausgangsposition ist 
 
  // Beide Motoren initial anhalten 
  stopMotors(); 
} 
 
////////////////////////////////////////////// 
// Hauptschleife // 
//////////    digitalWrite(Motor_rechts_B, HIGH); 
  } else { 
    digitalWrite(Motor_rechts_A, HIGH); 
    digitalWrite(Motor_rechts_B, LOW); 
  } 
} 
 
// Funktion zur Steuerung der Schussmechanik 
void shootBall() { 
  kickServo.write(90); // Servo in Schussposition drehen 
  delay(500);          // Position kurz halten 
  kickServo.write(0);  // In Ausgangsposition zurückkehren 
} //////////////////////////////////// 
void loop() { 
  RemoteXY_Handler(); 
 
  // Sensorwerte auslesen 
  frontSensorValue = analogRead(frontSensorPin); 
  rearSensorValue = analogRead(rearSensorPin); 
 
  Serial.print("Frontsensor: "); 
  Serial.print(frontSensorValue); 
  Serial.print("  Rücksensor: "); 
  Serial.println(rearSensorValue); 
 
  // Überprüfen, ob der Frontsensor eine schwarze Linie erkennt 
  if (frontSensorValue >= threshold) { 
    stopMotors(); 
    autoBack();  
    stopMotors(); 
  } else if (rearSensorValue >= threshold) { 
    stopMotors(); 
    moveForward(); 
    stopMotors(); 
  } else { 
    controlMotorsWithJoystick(RemoteXY.joystick_x, RemoteXY.joystick_y); 
  } 
 
  // Überprüfen, ob der Schussbefehl vorliegt 
  if (RemoteXY.button_1 == 1 && !previousButton1State) {  
    // Nur ausführen, wenn Knopf gedrückt ist und zuvor nicht gedrückt war 
    shootBall(); 
  } 
  // Aktualisieren des vorherigen Zustands des Knopfes 
  previousButton1State = (RemoteXY.button_1 == 1); 
} 
 
////////////////////////////////////////////// 
// Motor- und Schussfunktionen // 
////////////////////////////////////////////// 
 
// Funktion zum Stoppen beider Motoren 
void stopMotors() { 
  digitalWrite(Motor_links_A, LOW); 
  digitalWrite(Motor_links_B, LOW); 
  digitalWrite(Motor_rechts_A, LOW); 
  digitalWrite(Motor_rechts_B, LOW); 
} 
 
// Funktion, um das Auto rückwärts fahren zu lassen 
void autoBack() { 
  digitalWrite(Motor_links_A, HIGH); 
  digitalWrite(Motor_rechts_A, HIGH); 
  delay(143);  
} 
 
// Funktion, um das Auto vorwärts fahren zu lassen 
void moveForward() { 
  digitalWrite(Motor_links_A, LOW); 
  digitalWrite(Motor_links_B, HIGH); 
  digitalWrite(Motor_rechts_A, LOW); 
  digitalWrite(Motor_rechts_B, HIGH); 
  delay(143);  
} 
 
// Funktion zur Steuerung der Motoren basierend auf Joystick-Eingaben 
void controlMotorsWithJoystick(int8_t x, int8_t y) { 
  int speedLeft = 0; 
  int speedRight = 0; 
   
  if (x == 0 && y == 0) { 
    stopMotors(); 
    return; 
  } 
 
  int motorSpeed = map(abs(y), 0, 100, 0, 255); 
  int turnSpeed = map(abs(x), 0, 100, 0, 255); 
 
  if (y > 0) { 
    speedLeft = motorSpeed; 
    speedRight = motorSpeed; 
  } else if (y < 0) { 
    speedLeft = -motorSpeed; 
    speedRight = -motorSpeed; 
  } 
 
  if (x < 0) { 
    speedLeft += turnSpeed; 
    speedRight -= turnSpeed; 
  } else if (x > 0) { 
    speedLeft -= turnSpeed; 
    speedRight += turnSpeed; 
  } 
 
  if (speedLeft >= 0) { 
    digitalWrite(Motor_links_A, LOW); 
    digitalWrite(Motor_links_B, HIGH); 
  } else { 
    digitalWrite(Motor_links_A, HIGH); 
    digitalWrite(Motor_links_B, LOW); 
  } 
 
  if (speedRight >= 0) { 
    digitalWrite(Motor_rechts_A, LOW);
