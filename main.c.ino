#include "bsp.h"

#ifdef ARDUINO_BSP
   #define LED_PIN 13
  #define buttonPin 2
  #define adc_pin A0
  volatile bool buttonPressed = false;
  volatile bool print_flag = false;
#endif

#ifdef ESP_BSP
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/semphr.h>

  #define LED_PIN 5
  #define BTN_PIN 2
  #define ADC_PIN 34

  bool system_on = false; // Estado del sistema
  int button_press_count = 0; // Contador de presiones del botón
  bool adc_active = false; // Estado del ADC
  unsigned long last_button_press_time = 0; // Tiempo del último pulso del botón

  SemaphoreHandle_t xButtonSemaphore;
  SemaphoreHandle_t xAdcSemaphore;
#endif

void setup() {
  #ifdef ARDUINO_BSP
    gpio_config(LED_PIN, OUTPUT); // Configura el pin del LED como salida
    gpio_config(buttonPin, INPUT_PULLUP); // Configuramos el pin del botón como entrada con resistencia pull-up interna
    gpio_config(adc_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
    serial_init(9600);
  #endif

  #ifdef ESP_BSP
    gpio_config(LED_PIN, OUTPUT); // Configura el pin del LED como salida
    gpio_config(BTN_PIN, INPUT); // Configura el pin del botón como entrada
    serial_init(9600); // Inicializa la comunicación serial
    adc_init(); // Inicializa el ADC

    xButtonSemaphore = xSemaphoreCreateBinary();
    xAdcSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(button_task, "Button Task", 2048, NULL, 1, NULL);
    xTaskCreate(led_task, "LED Task", 2048, NULL, 1, NULL);
    xTaskCreate(adc_task, "ADC Task", 2048, NULL, 1, NULL);
  #endif
}

void loop() {
  #ifdef ARDUINO_BSP 
    if(buttonPressed == true){
      
      gpio_write(LED_PIN, HIGH);

      while(buttonPressed == true){print_adc();} 
    }

    gpio_write(LED_PIN, LOW);
    serial_write("No_Disponible");

    while(buttonPressed != true);
  #endif

  #ifdef ESP_BSP
    vTaskDelay(portMAX_DELAY); // Mantiene el loop en un estado inactivo
  #endif

}

#ifdef ARDUINO_BSP

  void print_adc(){
    int adc_lec[10];
    bool adc_conected = true;

    for(int i = 0; i<10; i++){
      adc_lec[i] = adc_read(adc_pin);
      if(i > 0){
        if( ((adc_lec[i-1] - adc_lec[i]) > 2) || ((adc_lec[i-1] - adc_lec[i]) < -2) || ((adc_lec[i-1] - adc_lec[i]) == 1023) ){
          adc_conected = false;
        }
      }
    }

    if(adc_conected){
      print_flag = false;
      serial_write(adc_lec[9]);
    }

    if(print_flag == false && adc_conected == false){
      print_flag = true;
      serial_write("ADC not conected");
    }

    delay(200);
  }

  void handleButtonPress() {
    buttonPressed = !buttonPressed;
  }
#endif

#ifdef ESP_BSP
  void button_task(void *pvParameters) {
      static bool last_button_state = LOW; // Estado anterior del botón
      bool current_button_state;

      for (;;) {
          current_button_state = gpio_read(BTN_PIN);

          // Detecta el cambio de estado del botón
          if (current_button_state == HIGH && last_button_state == LOW && (millis() - last_button_press_time) > 50) {
              last_button_press_time = millis();
              button_press_count++;

              xSemaphoreGive(xButtonSemaphore);
          }

          last_button_state = current_button_state; // Actualiza el estado anterior del botón

          vTaskDelay(pdMS_TO_TICKS(50)); // Espera 50 ms
      }
  }

  void led_task(void *pvParameters) {
      for (;;) {
          if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY)) {
              system_on = (button_press_count % 2 == 1);

              // Enciende o apaga el sistema
              if (system_on) {
                  gpio_write(LED_PIN, HIGH); // Enciende el LED
                  serial_write("System ON\n");
                  adc_active = true; // Activa el ADC
              } else {
                  gpio_write(LED_PIN, LOW); // Apaga el LED
                  serial_write("System OFF\n");
                  adc_active = false; // Desactiva el ADC
              }
          }
          vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100 ms
      }
  }

 void adc_task(void *pvParameters) {
    for (;;) {
        if (system_on) {
            int adc_value = adc_read(ADC_PIN);
            
            // Verifica si el valor leído es 0, indicando una posible desconexión
            if (adc_value == 0) {
                serial_write("ADC Desconectado\n");
            } else {
                String message = "ADC Value: " + String(adc_value) + "\n";
                serial_write(message.c_str());
            }
        } else {
            serial_write("No_Disponible\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo
    }
   }
#endif


