#include <Arduino.h>

//--------------------
//CONFIG
//--------------------

//Sequence of traffic lights
uint8_t sequence[][2] = {{0, 1}, {2, 3}};

//Red, Yellow/Amber, Green
uint8_t pins[][3]= {{13, 12, 11}, {10, 9, 8}, {5, 6, 7}, {2, 3, 4}};

//Lights length
#define RED_LENGTH 2000 //Between greens
#define GREEN_LENGTH 5000

#define YELLOW_LENGTH 2000
//--------------------
//END OF CONFIG
//--------------------

static uint8_t step_num = 0;

#define NUM_STEPS sizeof(sequence)/sizeof(sequence[0])

void lights_on(uint8_t* step);
void lights_off(uint8_t* step);

void setup() {
  uint8_t i = 0;
  uint8_t j = 0;

  for (i=0; i<(sizeof(pins)/sizeof(pins[0])); i++){
    for (j=0; j<3; j++){
      pinMode(pins[i][j],OUTPUT);
    }
  }
  
  for (i=0; i<(sizeof(pins)/sizeof(pins[0])); i++){
    digitalWrite(pins[i][0],HIGH);
  }
}

void loop() {
  uint8_t* current_step = sequence[step_num];

  lights_on(current_step);
  delay(GREEN_LENGTH);
  lights_off(current_step);
  delay(RED_LENGTH);

  if (step_num >= NUM_STEPS-1) step_num=0;
  else step_num++;
}

void lights_on(uint8_t* step){
  digitalWrite(pins[step[0]][1],HIGH); //Yellow/Amber - ON
  digitalWrite(pins[step[1]][1],HIGH);
  delay(YELLOW_LENGTH);
  digitalWrite(pins[step[0]][0],LOW); //Yellow/Amber, Red - OFF, Green - ON
  digitalWrite(pins[step[1]][0],LOW);
  digitalWrite(pins[step[0]][1],LOW);
  digitalWrite(pins[step[1]][1],LOW);
  digitalWrite(pins[step[0]][2],HIGH);
  digitalWrite(pins[step[1]][2],HIGH);
}

void lights_off(uint8_t* step){
  digitalWrite(pins[step[0]][2],LOW); //Green - OFF, Yellow/Amber - ON
  digitalWrite(pins[step[1]][2],LOW);
  digitalWrite(pins[step[0]][1],HIGH);
  digitalWrite(pins[step[1]][1],HIGH);
  delay(YELLOW_LENGTH);
  digitalWrite(pins[step[0]][1],LOW); //Yellow/Amber - OFF, Red - ON
  digitalWrite(pins[step[1]][1],LOW);
  digitalWrite(pins[step[0]][0],HIGH);
  digitalWrite(pins[step[1]][0],HIGH);
}