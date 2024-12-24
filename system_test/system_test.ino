#define SAMPLE_RATE 300 // Sampling frequency (300 Hz)
#define SAMPLE_PERIOD (1000 / SAMPLE_RATE) // Time interval per sample in milliseconds

const int analogPin = A4; // Analog pin to monitor
const float referenceVoltage = 3.3; // Assume the reference voltage is 3.3V

unsigned long lastSampleTime = 0; // Last sampling time
int sampleCount = 0; // Count of samples in the current period
float voltageSum = 0; // Sum of all voltages in the current period
int adcSum = 0; // Sum of ADC values in the current period
int elapsedSeconds = 0; // Tracks the elapsed time in seconds

void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial);      // Wait for the serial port to initialize
  Serial.println("Starting A4 pin monitoring...");
}

void loop() {
  unsigned long currentTime = millis();

  // Sample the analog pin at the defined frequency
  if (currentTime - lastSampleTime >= SAMPLE_PERIOD) {
    lastSampleTime = currentTime; // Update the last sampling time

    // Read the analog value
    int analogValue = analogRead(analogPin);

    // Convert the analog value to voltage
    float voltage = (analogValue / 1023.0) * referenceVoltage;

    // Accumulate the ADC value and voltage
    adcSum += analogValue;
    voltageSum += voltage;
    sampleCount++;
  }

  // Output the average every second (after 300 samples)
  if (sampleCount >= SAMPLE_RATE) {
    // Calculate averages
    float averageVoltage = voltageSum / sampleCount; // Average voltage
    int averageADC = adcSum / sampleCount; // Average ADC value

    // Output to Serial Plotter (only ADC value for plotting)
    Serial.println(averageADC);

    // Output to Serial Monitor (both ADC and voltage)
    Serial.print("Time: ");
    Serial.print(elapsedSeconds);
    Serial.print(" s, Average ADC: ");
    Serial.print(averageADC);
    Serial.print(", Average Voltage: ");
    Serial.print(averageVoltage);
    Serial.println(" V");

    // Reset accumulators for the next period
    voltageSum = 0;
    adcSum = 0;
    sampleCount = 0;

    // Increment elapsed time
    elapsedSeconds++;
  }
}
