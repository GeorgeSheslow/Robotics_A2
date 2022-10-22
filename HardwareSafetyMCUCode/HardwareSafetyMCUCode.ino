#define button_pin 7
#define IR_pin A5

const long pub_interval = 250;
unsigned long pub_previousMillis = 0;

int button_state = 0;
int current_state = 0;

int IR_data;
const long IR_interval = 50;
unsigned long IR_previousMillis = 0;

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(INPUT,button_pin);
}

// The loop routine runs over and over again forever:
void loop() {

// Estop Button
if (digitalRead(button_pin)){
  current_state = 1;
}
if (!digitalRead(button_pin) && current_state == 1){
  current_state = 0;
  button_state ^= 1;
}

// IR data
if (millis() - IR_previousMillis >= IR_interval) {
    IR_previousMillis = millis();
    IR_data = analogRead(IR_pin);
}

// publishing data
if (millis() - pub_previousMillis >= pub_interval) {
    pub_previousMillis = millis();
    // IR data
    Serial.print("IR ");
    Serial.print(IR_data);
    Serial.write(13);
    Serial.write(10);
  
    // toggle Estop button
    Serial.print("EStop ");
    Serial.print(button_state);
    Serial.write(13);
    Serial.write(10);
}

}
