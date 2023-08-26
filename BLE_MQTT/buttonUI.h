class Button {
public:
  Button(int _pin) {
    // buttonInstance = this; // Set the instance pointer
    pin = _pin;
    pinMode(pin, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(pin), handleButtonPressStatic, FALLING);
  
  }
  unsigned long pressStartTime;
  int pin;
  bool b_state;

  bool isPressed(){
    return !digitalRead(pin);
  }
//   void loop() {
//     if (buttonPressed) {
//       unsigned long currentTime = millis();
//       unsigned long pressDuration = currentTime - pressStartTime;

//       if (pressDuration >= longPressThreshold && pressDuration < (longPressThreshold + 3000)) {
//         onPressFunction();
//       }

//       buttonPressed = false;
//     }
//   }

// private:
//   static Button *buttonInstance; // Pointer to the instance of the class

//   static void handleButtonPressStatic() {
//     if (buttonInstance) {
//       buttonInstance->handleButtonPress();
//     }
//   }

//   void handleButtonPress() {
//     buttonPressed = true;
//     pressStartTime = millis();
//   }
};


