#include <vector>

#define BUZZER_PIN  14
#define TIME_UNIT   200

/* We use cut morse code numbers for beeps
 * 0: T (-)
 * 1: A (.-)
 * 2: U (..-)
 * 3: V (...-)
 * 4: ....-
 * 5: E (.)
 * 6: -....
 * 7: B (-...)
 * 8: D (-..)
 * 9: N (-.)
 */

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void buzzShort(int frequency) {
  tone(BUZZER_PIN, frequency);
  delay(TIME_UNIT);
  noTone(BUZZER_PIN);
  delay(TIME_UNIT);
}
void buzzLong(int frequency) {
  tone(BUZZER_PIN, frequency);
  delay(TIME_UNIT*3);
  noTone(BUZZER_PIN);
  delay(TIME_UNIT);
}

void buzzAltitude(uint16_t altitude) {
  // Change altitude to a base 10 number
  std::vector<int> digits;
  while (altitude > 0) {
    Serial.println(altitude % 10);
    digits.push_back(altitude % 10);
    altitude /= 10;
  }

  int frequency = 1000;

  // digits are collected in reverse order, so loop over backwards
  for(int j=digits.size() - 1; j>=0; j--) {
    // delay in between digits
    delay(TIME_UNIT*3);
    // For 0-4 we do n-1 dots then dash
    if (digits[j] < 5) {
      for (int i=0; i<digits[j]-1; i++)
        buzzShort(frequency);
      buzzLong(frequency);
    }
    // For 5 we do one dot
    else if (digits[j] < 6) {
      buzzShort(frequency);
    }
    // For 6-9 we do dash then 10-n dots
    else {
      buzzLong(frequency);
      for (int i=0; i<10-digits[j]; i++)
        buzzShort(frequency);
    }
    // Increase the frequency by a major third
    // frequency *= 1.25;
  }

  delay(TIME_UNIT*7);
}